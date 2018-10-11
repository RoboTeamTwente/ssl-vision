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
#include <utility>

using namespace cv;

class ArucoMarkerfinder {
public:
    ArucoMarkerfinder();

    void findMarkers(Mat image, std::vector<int> &markerIds, std::vector<int> &markerX, std::vector<int> &markerY,
                     std::vector<float> &markerTheta);

private:
    int g_skipPixels = 10;                      // skip this many pixels in the initial blobfinding to increase performance
    Vec3b g_LowerWhiteMargin = {90, 90, 90};      // minimum color threshold for a pixel to be determined 'white'
    Vec3b g_upperWhiteMargin = {255, 255, 255};   // maximum color threshold
    Vec3b g_deltaWhiteMargin = {15, 15, 15};           // after one pixel has been found, lower the minimum threshold by this amount
    float g_maxAngleDeviation = 0.1;
    float g_maxLengthDeviation = 0.1;
    int g_minMarkerPixels = 100;                // minimum white pixels required to determine marker
    const int ARUCOSIZE = 3;                    // gridsize of aruco data
    const int NARUCOMARKERS = 32;               // amount of markers
    const int MAXRECURSION = 25000;             // hotfix for segmentation faults (139) at recursion depth 26190..


    bool isColor(Vec3b color, Vec3b lowerBound, Vec3b upperBound);

    int findFurthestPixel(int xRelative, int yRelative, int startIndex, int endIndex, std::vector<int> &x,
                          std::vector<int> &y);

    std::vector<int> findOppositeCorner(int xRelative, int yRelative, int startIndex, int endIndex, std::vector<int> &x,
                           std::vector<int> &y,
                           Vec3b &lowerBound, Vec3b &upperBound, Mat image, Vec3b &setColor);

    void createVectors(std::vector<int> &x, std::vector<int> &y, std::vector<int> &index,
                       std::vector<int> &v0, std::vector<int> &v1, std::vector<int> &v2, std::vector<int> &v3);

    int dotProduct(std::vector<int> &v1, std::vector<int> &v2);

    float calcVectorLength(std::vector<int> &v1);

    float calcCosAngle(float dot, float v0Length, float v1Length);

    void findWhiteBlob(int i, int j, std::vector<int> &x, std::vector<int> &y, std::vector<int> &index, Mat image);

    void groupWhitePixels(int i, int j, std::vector<int> &x, std::vector<int> &y, Vec3b lowerBound, Vec3b upperBound,
                          Mat whitePixels, Vec3b setColor, int iteration);

    bool findMarkerData(std::vector<bool> &markerData, std::vector<int> &u, std::vector<int> &v,
    std::vector<int> &uu, std::vector<int> &vv, std::vector<int> &corners, Mat image);

    bool checkIfSquareBlob(std::vector<int> &x, std::vector<int> &y, int startIndex, int endIndex, Mat image,
            std::vector<int> &corners, std::vector<int> &u, std::vector<int> &v, std::vector<int> &uu, std::vector<int> &vv);

    bool findMarkerId(std::vector<bool> &resultData, std::vector<float> &posRotId, std::vector<int> &corners);

};


#endif //SSL_VISION_ARUCO_MARKERFINDER_H
