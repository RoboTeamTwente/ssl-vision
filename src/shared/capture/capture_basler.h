/*
 * capture_basler.h
 *
 *  Created on: Nov 21, 2016
 *      Author: Dennis
 */

#ifndef CAPTURE_BASLER_H_
#define CAPTURE_BASLER_H_

#include "captureinterface.h"
#include <pylon/PylonIncludes.h>
#include <pylon/PylonBase.h>
#include <pylon/PylonImage.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <sys/time.h>
#include "VarTypes.h"

#ifndef VDATA_NO_QT
#include <QMutex>
#endif

class BaslerInitManager {
public:
	static void register_capture();
	static void unregister_capture();
private:
	static int count;
};

struct CameraSorter {
	enum SerialCheckStatus { TODO, NO, YES };
	static enum SerialCheckStatus serialCheck;
	bool operator()(const Pylon::CDeviceInfo& a, const Pylon::CDeviceInfo& b) {
		if (serialCheck == TODO) {
			serialCheck = a.IsSerialNumberAvailable() ? YES : NO;
			if (serialCheck == NO) {
				std::cerr << "[Basler] Warning: serial numbers not available. Sorting by possibly non-persistent user-defined names instead.\n";
			}
		}
		if (serialCheck == YES) {
			return a.GetSerialNumber() < b.GetSerialNumber();
		}
		return a.GetUserDefinedName() < b.GetUserDefinedName();
	}
};

#ifdef VDATA_NO_QT
class CaptureBasler: public CaptureInterface

#else
class CaptureBasler: public QObject, public CaptureInterface {
public:
	Q_OBJECT

    public slots:
    void changed(VarType * group);
private:
	QMutex mutex;
#endif

public:
#ifndef VDATA_NO_QT
	CaptureBasler(VarList* _settings=0, QObject* parent=0);
    void mvc_connect(VarList * group);
#else
	CaptureBasler(VarList * _settings=0);
#endif
	~CaptureBasler();

	bool startCapture();

	bool stopCapture();

	bool isCapturing() { return isGrabbing; };

	RawImage getFrame();

	void releaseFrame();

	string getCaptureMethodName() const;

	bool copyAndConvertFrame(const RawImage & src, RawImage & target);

	void readAllParameterValues();

	void writeParameterValues(VarList* vars);

private:
	bool isGrabbing;
	Pylon::CBaslerGigEInstantCamera* camera;
	Pylon::CImageFormatConverter converter;
	int currentID;
  	unsigned char* lastBuf;

  	VarList* vars;
  	VarInt* vCameraID;
  	VarInt* vBalanceRatioRed;
  	VarInt* vBalanceRatioGreen;
  	VarInt* vBalanceRatioBlue;
  	VarBool* vAutoGain;
  	VarDouble* vGain;
  	VarBool* vEnableGamma;
  	VarDouble* vGamme;
  	VarDouble* vBlackLevel;
  	VarBool* vAutoExposure;
  	VarDouble* vManualExposure;
  	VarStringEnum* vColorMode;

  	void resetCamera(unsigned int newID);
  	bool stopGrabbing();
  	bool buildCamera();

// A slight blur helps to reduce noise and improve color recognition.
#ifdef OPENCV
  	static const double blur_sigma;
  	void gaussianBlur(RawImage& img);
    void contrast(RawImage& img, double factor);
    void sharpen(RawImage& img);
    void equalize(RawImage& img);
#endif
};

#endif
