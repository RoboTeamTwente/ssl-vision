//
// Created by Wouter Timmermans on 14-09-17.
//

#include "aruco_detector.h"

#define DEBUG

ArucoDetector::ArucoDetector() {

    totalTime = 0;
    totalIterations = 0;

    // Populate the camera optical parameters. These are unique per camera and should be recalibrated if the camera focus
    // changes
    cv::Ptr<cv::Mat> camPtr, distPtr;

    camPtr = cv::makePtr<cv::Mat>();
    distPtr = cv::makePtr<cv::Mat>();

    readCameraParameters(camPtr, distPtr);


    cameraMatrix = *camPtr;
    distCoeffs = *distPtr;

}


std::vector<PosRotId> ArucoDetector::performTrackingOnImage(cv::Mat image) {

    std::vector<PosRotId> result = std::vector<PosRotId>();

    if (!image.empty()) {
        dict_mutex.lock();

        std::vector<int> markerIds;
        std::vector<int> markerX;
        std::vector<int> markerY;
        std::vector<float> markerTheta;

        finder.setG_LowerWhiteMargin(lower_white_margin);
        finder.setG_upperWhiteMargin(upper_white_margin);
        finder.setG_deltaWhiteMargin(delta_margin);
        finder.findMarkers(image, markerIds, markerX, markerY, markerTheta);

        dict_mutex.unlock();

        if (!markerIds.empty()) {
#ifdef DEBUG
            printf("%-12s%-12s%-12s%-12s\n", "ID", "x (pixels)", "y (pixels)", "angle (pi rad)");
#endif
            for (int i = 0; i < (int)markerIds.size(); i++) {
                int id = markerIds[i];
                int x = markerX[i];
                int y = markerY[i];
                float angle = markerTheta[i];

                PosRotId posRot = PosRotId(id, x, y, angle);
                result.insert(result.end(), posRot);

#ifdef DEBUG
                    std::string strID = std::to_string( id );
                    std::string strX = std::to_string( x );
                    std::string strY = std::to_string( y );
                    std::string strT = std::to_string( (angle) );

                    char const *idid = strID.c_str();
                    char const *xx = strX.c_str();
                    char const *yy = strY.c_str();
                    char const *theta = strT.c_str();

                    printf("%-12s%-12s%-12s%-12s\n", idid, xx, yy, theta);
#endif
            }
        }
    }
    return result;
}

void ArucoDetector::readCameraParameters(cv::Ptr<cv::Mat> cameraMatrix, cv::Ptr<cv::Mat> distCoeffs) {
    //TODO: read these paramaters from a file
    double distArr[] = {-8.2549974582253244e-02, -1.5181783979810068e+00,
                        -4.9753678549509261e-04, 4.7107295904877293e-03,
                        4.0871908247047974e+00};
    cv::Matx33d camMatx(8.6298813107174658e+02, 0., 6.7620649647659229e+02, 0.,
                        8.6553882635457478e+02, 4.7027690275075901e+02, 0., 0., 1.);
    cv::Mat camMat = cv::Mat(cv::Size(3, 3), CV_64FC1);
    camMat = cv::Mat(camMatx * camMatx.t());
    cv::Mat distMat = cv::Mat(cv::Size(5, 1), CV_64FC1, distArr);
    *cameraMatrix = camMat;
    *distCoeffs = distMat;
}

void ArucoDetector::setLowerWhiteMargin(const unsigned char &blue, const unsigned char &green, const unsigned char &red) {
    ArucoDetector::lower_white_margin = {red, green, blue};
}

void ArucoDetector::setDeltaMargin(const unsigned char &delta_margin) {
    ArucoDetector::delta_margin = {delta_margin, delta_margin, delta_margin};
}

void ArucoDetector::setUpperWhiteMargin() {
    ArucoDetector::upper_white_margin = {255,255,255};
}

