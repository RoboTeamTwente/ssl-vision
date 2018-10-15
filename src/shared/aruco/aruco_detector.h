//
// Created by Wouter Timmermans on 14-09-17.
//

#ifndef SURFBOTTRACKING_TRACKER_H
#define SURFBOTTRACKING_TRACKER_H

#include <vector>
#include <string>
#include "opencv2/opencv.hpp"
#include <cmath>
#include <mutex>
#include "pos_rot_id.h"
#include "aruco_markerfinder.h"
#include <unistd.h>

class ArucoDetector {

public:

    /// Initialise the tracker. A tracker can locate the location and rotation of markers on a flat surface.
    /// Two markers serve as boundaries for the area, the origin and the limit. The origin serves as point 0,0 and the
    /// limit as point RESULOTION,RESOLUTION e.g. 1000,1000. The rotation that performTracking returns is relative to
    /// the rotation of origin.
    /// \param originID id of marker that serves as origin.
    /// \param limitID id of marker that serves as limit.
    /// \param camId the OpenCV camera id to use for the tracking
    explicit ArucoDetector();


    /// Performs the tracking and return the PosRots in a vector. Performs the tracking for one frame of the input device.
    /// \param showDebug if true shows camera view with marker angles and observations on cout
    /// \returns A vector containing PosRotIds for each marker that has been observed in this frame.
    //std::vector<PosRotId> performTracking(bool showDebug);

    std::vector<PosRotId> performTrackingOnImage(cv::Mat image, bool showDebug);
    void setLowerWhiteMargin(const unsigned char &blue, const unsigned char &green, const unsigned char &red);
    void setDeltaMargin(const unsigned char &delta_margin);
    void setUpperWhiteMargin();
private:

    std::mutex dict_mutex;
    Vec3b upper_white_margin = {255,255,255};
    Vec3b lower_white_margin;
    Vec3b delta_margin;



    ArucoMarkerfinder finder;



    /// Fills cameraMatrix and distCoeffs with values. These values should come from the camera calibration performed
    /// with charuco boards.
    /// \param cameraMatrix camera properties matrix
    /// \param distCoeffs camera distortion matrix
    void readCameraParameters(cv::Ptr<cv::Mat> cameraMatrix, cv::Ptr<cv::Mat> distCoeffs);


    double totalTime;
    int totalIterations;


    cv::Mat cameraMatrix, distCoeffs;

};


#endif //SURFBOTTRACKING_TRACKER_H
