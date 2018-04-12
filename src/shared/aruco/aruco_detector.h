//
// Created by Wouter Timmermans on 14-09-17.
//

#ifndef SURFBOTTRACKING_TRACKER_H
#define SURFBOTTRACKING_TRACKER_H

#include <vector>
#include <string>
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include <cmath>
#include "pos_rot_id.h"

class ArucoDetector {

public:

    /// Initialise the tracker. A tracker can locate the location and rotation of markers on a flat surface.
    /// Two markers serve as boundaries for the area, the origin and the limit. The origin serves as point 0,0 and the
    /// limit as point RESULOTION,RESOLUTION e.g. 1000,1000. The rotation that performTracking returns is relative to
    /// the rotation of origin.
    /// \param originID id of marker that serves as origin.
    /// \param limitID id of marker that serves as limit.
    /// \param camId the OpenCV camera id to use for the tracking
    explicit ArucoDetector(int total_markers, int bits);


    /// Performs the tracking and return the PosRots in a vector. Performs the tracking for one frame of the input device.
    /// \param showDebug if true shows camera view with marker angles and observations on cout
    /// \returns A vector containing PosRotIds for each marker that has been observed in this frame.
    //std::vector<PosRotId> performTracking(bool showDebug);


    std::vector<PosRotId> performTrackingOnImage(cv::Mat image, bool showDebug);
    void setDictionaryProperties(int total_markers, int bits);
private:
    

    /// Transforms the observed marker position camera coordinate to a coordinate relative to origin and limit.
    /// returns [0,0] if the position is outside the area defined by the origin and limit marker.
    /// \param origin the position of the lower left corner of the origin marker.
    /// \param limit the position of the lower left corner of the limit marker.
    /// \param observedPos the observed positions of the corners of a marker in camera coordinates.
    /// \return the position relative to origin and limit of the observed marker.
    cv::Point2f
    calculatePosition(std::vector<cv::Point2f> observedPos);

    /// Fills cameraMatrix and distCoeffs with values. These values should come from the camera calibration performed
    /// with charuco boards.
    /// \param cameraMatrix camera properties matrix
    /// \param distCoeffs camera distortion matrix
    void readCameraParameters(cv::Ptr<cv::Mat> cameraMatrix, cv::Ptr<cv::Mat> distCoeffs);


	///returns the euclidean distance between two Point2f
	double euclideanDist(cv::Point2f a, cv::Point2f b);	
	
	///returns the average resolution between each edge of the marker, by giving the vector that contains the corners of the marker
	double getAvgRes(std::vector<cv::Point2f> corners);

    double totalTime;
    int totalIterations;

//    cv::Point2f origin;
//    cv::Point2f limit;
//    cv::Mat originRot;
//    cv::Mat limitRot;

    cv::Mat cameraMatrix, distCoeffs;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::Dictionary::create(20,3);
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
};


#endif //SURFBOTTRACKING_TRACKER_H
