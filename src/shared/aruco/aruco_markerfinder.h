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
    int g_skipPixels = 10;                  // skip this many pixels in the initial blobfinding to increase performance
    int g_whiteMargin = 155;                // minimum color threshold for a pixel to be determined 'white'
    int g_deltaWhiteMargin = 25;            // after one pixel has been found, lower the minimum threshold by this amount
    int g_minMarkerPixels = 500;            // minimum white pixels required to determine marker
    const int ARUCOSIZE = 3;                // gridsize of aruco data
    const int NARUCOMARKERS = 32;           // amount of markers
    const int MAXRECURSION = 25000;         // hotfix for segmentation faults at recursion depth 26190..


    bool isWhite(Vec3b &color, int white);
    void findWhitePixels(int i, int j, std::vector<int> &x, std::vector<int> &y, std::vector<int> &index, Mat image);
    void findWhiteSquares(int i, int j, std::vector<int> &x, std::vector<int> &y, Mat whitePixels, int iteration);
    bool findMarkerData(std::vector<int> &x, std::vector<int> &y, int sI, int eI, Mat image, std::vector<bool> &markerData, std::vector<float> &posRot);
    void findMarkerId(std::vector<bool> &resultData, std::vector<int> &markerIds);

    };


#endif //SSL_VISION_ARUCO_MARKERFINDER_H
