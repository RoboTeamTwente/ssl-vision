//
// Created by thijs on 2-10-18.
//

#ifndef SSL_VISION_ARUCO_MARKERFINDER_H
#define SSL_VISION_ARUCO_MARKERFINDER_H

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <iostream>
#include <vector>

using namespace cv;

class ArucoMarkerfinder {
public:
    ArucoMarkerfinder();
    void findMarkers(Mat image, std::vector<int> &markerIds, std::vector<int> &markerX, std::vector<int> &markerY, std::vector<int> &markerTheta);

private:
    int g_skipPixels = 10;
    int g_whiteMargin = 105;
    int g_deltaWhiteMargin = 15;
    int g_minMarkerPixels = 500;
    const int ARUCOSIZE = 3;
    const int NARUCOMARKERS = 32;
    const int MAXRECURSION = 25000;

    bool isWhite(Vec3b &color, int white);
    void findWhitePixels(int i, int j, std::vector<int> &x, std::vector<int> &y, std::vector<int> &index, Mat image);
    void findWhiteSquares(int i, int j, std::vector<int> &x, std::vector<int> &y, Mat whitePixels, int iteration);
    bool findMarkerData(std::vector<int> &x, std::vector<int> &y, int sI, int eI, Mat image, std::vector<int> &markerData, std::vector<float> &posRot);
    void findMarkerId(std::vector<int> &resultData, std::vector<int> &markerIds);

    };


#endif //SSL_VISION_ARUCO_MARKERFINDER_H
