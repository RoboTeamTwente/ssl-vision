/*
 * capture_basler.cpp
 *
 *  Created on: Nov 21, 2016
 *      Author: Dennis
 */

#include "capture_basler.h"

#include <vector>
#include <string>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <pylon/ThreadPriority.h>

#ifndef VDATA_NO_QT
#define MUTEX_LOCK mutex.lock()
#define MUTEX_UNLOCK mutex.unlock()
#else
#define MUTEX_LOCK
#define MUTEX_UNLOCK
#endif

int BaslerInitManager::count = 0;

void BaslerInitManager::register_capture() {
	if (count++ == 0) {
		Pylon::PylonInitialize();
		Pylon::SetRTThreadPriority(Pylon::GetCurrentThreadHandle(), 60);
	}
}

void BaslerInitManager::unregister_capture() {
	if (--count == 0) {
		Pylon::PylonTerminate();
	}
}

#ifndef VDATA_NO_QT
CaptureBasler::CaptureBasler(VarList* _settings, QObject* parent) :
		QObject(parent), CaptureInterface(_settings) {
#else
	CaptureBasler::CaptureBasler(VarList* _settings) : CaptureInterface(_settings) {
#endif
	isGrabbing = false;
	camera = NULL;
	converter.OutputPixelFormat = Pylon::PixelType_RGB8packed;
	lastBuf = NULL;

	settings->addChild(vars = new VarList("Capture Settings"));
	settings->removeFlags(VARTYPE_FLAG_HIDE_CHILDREN);
	vars->removeFlags(VARTYPE_FLAG_HIDE_CHILDREN);
	vColorMode = new VarStringEnum("color mode",
			Colors::colorFormatToString(COLOR_RGB8));
	vColorMode->addItem(Colors::colorFormatToString(COLOR_YUV422_UYVY));
	vColorMode->addItem(Colors::colorFormatToString(COLOR_RGB8));
	vars->addChild(vColorMode);

	vars->addChild(vCameraID = new VarInt("Camera ID", 0, 0, 3));

	vBalanceRatioRed = new VarInt("Balance Ratio Red", 64, 0, 255);
	vars->addChild(vBalanceRatioRed);

	vBalanceRatioGreen = new VarInt("Balance Ratio Green", 64, 0, 255);
	vars->addChild(vBalanceRatioGreen);

	vBalanceRatioBlue = new VarInt("Balance Ratio Blue", 64, 0, 255);
	vars->addChild(vBalanceRatioBlue);

	vAutoGain = new VarBool("auto gain", true);
	vars->addChild(vAutoGain);

	vGain = new VarDouble("gain", 300, 200, 1000);
	vars->addChild(vGain);

	vEnableGamma = new VarBool("enable gamma correction", false);
	vars->addChild(vEnableGamma);

	vGamme = new VarDouble("gamma", 1.0, 0, 2);
	vars->addChild(vGamme);

	vBlackLevel = new VarDouble("black level", 64, 0, 1000);
	vars->addChild(vBlackLevel);

	vAutoExposure = new VarBool("auto exposure", false);
	vars->addChild(vAutoExposure);

	vManualExposure = new VarDouble("manual exposure (μs)", 10000, 1000,
			30000);
	vars->addChild(vManualExposure);

	currentID = 0;

#ifndef VDATA_NO_QT
	mvc_connect(settings);
	mvc_connect(vars);
#endif
}

CaptureBasler::~CaptureBasler() {
	vars->deleteAllChildren();
}

bool CaptureBasler::buildCamera() {
	BaslerInitManager::register_capture();
	Pylon::DeviceInfoList devices;
	int amt = Pylon::CTlFactory::GetInstance().EnumerateDevices(devices);
	currentID = vCameraID->get();
    printf("Current camera id: %d\n", currentID);
	if (amt > currentID) {
		Pylon::CDeviceInfo info = devices[currentID];

		camera = new Pylon::CBaslerGigEInstantCamera(
				Pylon::CTlFactory::GetInstance().CreateDevice(info));
        printf("Opening camera %d...\n", currentID);
		camera->Open();
		camera->PixelFormat.SetValue(Basler_GigECamera::PixelFormat_BayerBG8, true);
        printf("Setting interpacket delay...\n");
		camera->GevSCPD.SetValue(600);
		camera->InternalGrabEngineThreadPriorityOverride = 99;
        printf("Done!\n");
		isGrabbing = true;
		return true;
	}
	return false;
}

bool CaptureBasler::startCapture() {
	MUTEX_LOCK;
	try {
		if (camera == NULL) {
			if (!buildCamera()) {
                // Did not make a camera!
				MUTEX_UNLOCK;
                return false;
            }
		}
		camera->StartGrabbing(Pylon::GrabStrategy_OneByOne);
	} catch (Pylon::GenericException& e) {
        printf("Pylon exception: %s", e.what());
        delete camera;
        camera = NULL;
        currentID = -1;
		MUTEX_UNLOCK;
        return false;
	} catch (...) {
		MUTEX_UNLOCK;
		throw;
	}
	MUTEX_UNLOCK;
	return true;
}

bool CaptureBasler::stopGrabbing() {
	if (isGrabbing) {
		camera->StopGrabbing();
		camera->Close();
		isGrabbing = false;
		return true;
	}
	return false;
}

bool CaptureBasler::stopCapture() {
	MUTEX_LOCK;
	bool stopped;
	try {
		stopped = stopGrabbing();
		if (stopped) {
			delete camera;
			camera = 0;
			BaslerInitManager::unregister_capture();
		}
	} catch (...) {
		MUTEX_UNLOCK;
		throw;
	}
	MUTEX_UNLOCK;
	return stopped;
}

void CaptureBasler::releaseFrame() {
	MUTEX_LOCK;
	try {
		if (lastBuf) {
			delete[] lastBuf;
			lastBuf = NULL;
		}
	} catch (...) {
		MUTEX_UNLOCK;
		throw;
	}
	MUTEX_UNLOCK;
}

RawImage CaptureBasler::getFrame() {
	MUTEX_LOCK;
	// Make blank image
	RawImage img;
	img.setWidth(0);
	img.setHeight(0);
	img.setColorFormat(COLOR_RGB8);
	try {
		Pylon::CGrabResultPtr grabResult;

		// Calculate the timestamp
		timeval tv;
		gettimeofday(&tv, 0);
		img.setTime((double) tv.tv_sec + (tv.tv_usec / 1000000));

		// Keep grabbing in case of partial grabs
		int fail_count = 0;
		while (fail_count < 10
				&& (!grabResult || !grabResult->GrabSucceeded())) {
			try {
				// Get an image, waiting at most 1000 ms, store it in grabResult
				camera->RetrieveResult(75, grabResult,
						Pylon::TimeoutHandling_ThrowException);
			} catch (Pylon::TimeoutException& e) {
				fprintf(stderr,
						"Timeout expired in CaptureBasler::getFrame: %s\n",
						e.what());
				MUTEX_UNLOCK;
				return img;
			}
			if (!grabResult) {
				// If at first you don't succeed...
				fail_count++;
				continue;
			}
			if (!grabResult->GrabSucceeded()) {
				// Something was stored in the grabResult, but something went
				// wrong anyway. Log it, and try again.
				fail_count++;
				fprintf(stderr,
						"Image grab failed in CaptureBasler::getFrame: %s\n",
						grabResult->GetErrorDescription().c_str());
			}
		}
		if (fail_count == 10) {
			fprintf(stderr,
					"Maximum retry count for image grabbing (%d) exceeded in capture_basler",
					fail_count);
			MUTEX_UNLOCK;
			return img;
		}
		Pylon::CPylonImage capture;

		// Convert to RGB8 format
		converter.Convert(capture, grabResult);

		// Set the basics, and copy the buffer into the image.
		img.setWidth(capture.GetWidth());
		img.setHeight(capture.GetHeight());
		unsigned char* buf = new unsigned char[capture.GetImageSize()];
		memcpy(buf, capture.GetBuffer(), capture.GetImageSize());
		//img.setData((unsigned char*) capture.GetBuffer());
		img.setData(buf);

		// Optional post-processing:
#ifdef OPENCV
		// equalize(img);
		// gaussianBlur(img);
        // contrast(img, 1.6);
        // sharpen(img);
#endif

		// Keep a pointer to the new, copied buffer to clear it later.
		lastBuf = img.getData();

		// Original buffer is not needed anymore, it has been copied to img
		grabResult.Release();
	} catch (Pylon::GenericException& e) {
		fprintf(stderr, "Exception while grabbing a frame: %s\n", e.what());
		MUTEX_UNLOCK;
		throw;
	} catch (...) {
		// Make sure the mutex is unlocked before propagating
        printf("Uncaught exception!\n");
		MUTEX_UNLOCK;
		throw;
	}
	MUTEX_UNLOCK;
	return img;
}

string CaptureBasler::getCaptureMethodName() const {
	return "Basler";
}

bool CaptureBasler::copyAndConvertFrame(const RawImage & src,
		RawImage & target) {
	MUTEX_LOCK;
	try {
		target.ensure_allocation(COLOR_RGB8, src.getWidth(), src.getHeight());
		target.setTime(src.getTime());
		memcpy(target.getData(), src.getData(), src.getNumBytes());
	} catch (...) {
		MUTEX_UNLOCK;
		throw;
	}
	MUTEX_UNLOCK;
	return true;
}

void CaptureBasler::readAllParameterValues() {
	MUTEX_LOCK;
	try {
		if (!camera)
			// We have no active camera, so we can't read any settings
			return;

		if (!camera->IsOpen()) {
			// We have a reference to a camera, but somehow no active connection.
			// So we open one now.
			camera->Open();
		}

		camera->BalanceRatioSelector.SetValue(
				Basler_GigECamera::BalanceRatioSelector_Red);
		vBalanceRatioRed->setInt(camera->BalanceRatioRaw.GetValue());
		camera->BalanceRatioSelector.SetValue(
				Basler_GigECamera::BalanceRatioSelector_Green);
		vBalanceRatioGreen->setInt(camera->BalanceRatioRaw.GetValue());
		camera->BalanceRatioSelector.SetValue(
				Basler_GigECamera::BalanceRatioSelector_Blue);
		vBalanceRatioBlue->setInt(camera->BalanceRatioRaw.GetValue());

		vAutoGain->setBool(camera->GainAuto.GetValue() == Basler_GigECamera::GainAuto_Continuous);
		vGain->setDouble(camera->GainRaw.GetValue());
		vEnableGamma->setBool(camera->GammaEnable.GetValue());
		vGamme->setDouble(camera->Gamma.GetValue());

		vAutoExposure->setBool(camera->ExposureAuto.GetValue() == Basler_GigECamera::ExposureAuto_Continuous);
		vManualExposure->setDouble(camera->ExposureTimeAbs.GetValue());
	} catch (const Pylon::GenericException& e) {
		fprintf(stderr, "Exception reading parameter values: %s\n", e.what());
		MUTEX_UNLOCK;
		return;
	} catch (...) {
		MUTEX_UNLOCK;
		throw;
	}
	MUTEX_UNLOCK;
}

void CaptureBasler::resetCamera(unsigned int newID) {
	bool restart = isGrabbing;
	if (restart) {
		stopCapture();
	}
	currentID = newID;
	if (restart) {
		startCapture();
	}
}

void CaptureBasler::writeParameterValues(VarList* vars) {
	if (vars != this->settings) {
		return;
	}

	MUTEX_LOCK;
	try {
		// See if we need to start grabbing from a different camera,
		// and restart the camera if so.
		int newID = vCameraID->get();
		if (newID != currentID) {
			MUTEX_UNLOCK;
			resetCamera(vCameraID->get()); // locks itself
			MUTEX_LOCK;
		}

        if (camera != NULL) {

        	// Make sure we have an active connection first.
        	if (!camera->IsOpen()) {
        		camera->Open();
        	}

            camera->BalanceRatioSelector.SetValue(
                    Basler_GigECamera::BalanceRatioSelector_Red);
            camera->BalanceRatioRaw.SetValue(vBalanceRatioRed->get());
            camera->BalanceRatioSelector.SetValue(
                    Basler_GigECamera::BalanceRatioSelector_Green);
            camera->BalanceRatioRaw.SetValue(vBalanceRatioGreen->get());
            camera->BalanceRatioSelector.SetValue(
                    Basler_GigECamera::BalanceRatioSelector_Blue);
            camera->BalanceRatioRaw.SetValue(vBalanceRatioBlue->get());

            if (vAutoGain->getBool()) {
                camera->GainAuto.SetValue(Basler_GigECamera::GainAuto_Continuous);
            } else {
                camera->GainAuto.SetValue(Basler_GigECamera::GainAuto_Off);
                camera->GainRaw.SetValue(vGain->getInt());
            }

            if (vEnableGamma->getBool()) {
                camera->GammaEnable.SetValue(true);
                camera->Gamma.SetValue(vGamme->getDouble());
            } else {
                camera->GammaEnable.SetValue(false);
            }

            if (vAutoExposure->getBool()) {
                camera->ExposureAuto.SetValue(
                        Basler_GigECamera::ExposureAuto_Continuous);
            } else {
                camera->ExposureAuto.SetValue(Basler_GigECamera::ExposureAuto_Off);
                camera->ExposureTimeAbs.SetValue(vManualExposure->getDouble());
            }
        }
	} catch (const Pylon::GenericException& e) {
		MUTEX_UNLOCK;
		fprintf(stderr, "Error writing parameter values: %s\n", e.what());
		throw;
	} catch (...) {
		MUTEX_UNLOCK;
		throw;
	}
	MUTEX_UNLOCK;
}

#ifdef OPENCV
inline void CaptureBasler::gaussianBlur(RawImage& img) {
	cv::Mat cv_img(img.getHeight(), img.getWidth(), CV_8UC3, img.getData());
	cv::GaussianBlur(cv_img, cv_img, cv::Size(), blur_sigma);
}

void CaptureBasler::contrast(RawImage& img, double factor) {
	cv::Mat cv_img(img.getHeight(), img.getWidth(), CV_8UC3, img.getData());
    for (int y = 0; y < cv_img.rows; ++y) {
        for (int x = 0; x < cv_img.cols; ++x) {
            for (int i = 0; i < 3; ++i) {
                uint8_t channel = cv_img.at<cv::Vec3b>(y, x)[i];
                int newChannel = channel * factor;
                if (newChannel > 255) newChannel = 255;
                cv_img.at<cv::Vec3b>(y, x)[i] = newChannel;
            }
        }
    }
}

void CaptureBasler::sharpen(RawImage& img) {
	cv::Mat cv_img(img.getHeight(), img.getWidth(), CV_8UC3, img.getData());
	cv::Mat cv_img_copy = cv_img.clone();
	cv::GaussianBlur(cv_img_copy, cv_img_copy, cv::Size(), 3);
    cv::addWeighted(cv_img, 2.5, cv_img_copy, -1.5, 0, cv_img);
}

void CaptureBasler::equalize(RawImage& img) {
	cv::Mat original(img.getHeight(), img.getWidth(), CV_8UC3, img.getData());
	cv::Mat equalized(img.getHeight(), img.getWidth(), CV_8UC3, new signed char[img.getNumBytes()]);

	std::vector<cv::Mat> channels;
	cv::cvtColor(original, equalized, CV_RGB2YCrCb);
	cv::split(equalized, channels);
	cv::equalizeHist(channels[0], channels[0]);
	cv::merge(channels, equalized);
	cv::cvtColor(equalized, equalized, CV_YCrCb2RGB);
	img.setData(equalized.data);
}

#endif

#ifndef VDATA_NO_QT
void CaptureBasler::mvc_connect(VarList * group) {
	vector<VarType *> v = group->getChildren();
	for (unsigned int i = 0; i < v.size(); i++) {
		connect(v[i],SIGNAL(wasEdited(VarType *)),group,SLOT(mvcEditCompleted()));
	}
	connect(group,SIGNAL(wasEdited(VarType *)),this,SLOT(changed(VarType *)));
}

void CaptureBasler::changed(VarType * group) {
	if (group->getType() == VARTYPE_ID_LIST) {
		writeParameterValues(dynamic_cast<VarList*>(group));
	}
}
#endif
