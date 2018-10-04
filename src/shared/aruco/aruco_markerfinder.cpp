//
// Created by thijs on 2-10-18.
//

#include "aruco_markerfinder.h"

ArucoMarkerfinder::ArucoMarkerfinder() = default;


bool ArucoMarkerfinder::isWhite(Vec3b &color, int white) {
    bool pixelIsWhite = true;
    for (int c = 0; c <= 2; c++) {
        if (color[c] < white) {
            pixelIsWhite = false;
        }
    }
    return pixelIsWhite;
}

/// Finds pixels
void ArucoMarkerfinder::findWhitePixels(int i, int j, std::vector<int> &x, std::vector<int> &y, std::vector<int> &index, Mat image) {
    // assume the pixel is white, unless proven otherwise.. if white -> put data in new matrix and find the surrounding pixels.

    Vec3b color = image.at<Vec3b>(i,j);

    if ( isWhite(color, g_whiteMargin) ) {

        findWhiteSquares(i,j,x,y,image,0);
        index.push_back((int) x.size());

    }
}

void ArucoMarkerfinder::findWhiteSquares(int i, int j, std::vector<int> &x, std::vector<int> &y, Mat image, int iteration) {
    // push the current pixel data for this blob/square into a vector
    x.push_back(i);
    y.push_back(j);

    // 'itColorFactor' and 'd' is used for visualization of the recursive pixel checking
    Vec3b green = {0,255,0};

    if (iteration == 0) {
        // set the first pixel in the square to white
        Vec3b grey = {70,60,70};
        image.at<Vec3b>(i,j) = grey;
    }

    // if whe are not at the edge.. check left,up,right,down for pixels in the current square
    if (iteration < MAXRECURSION && (i > 1) && (j > 1) && (i < image.rows-1) && (j < image.cols-1) ) {

        Vec3b iMinColor = image.at<Vec3b>(i - 1, j);
        if ( isWhite(iMinColor, g_whiteMargin - g_deltaWhiteMargin) ) {
            image.at<Vec3b>(i - 1, j) = green;
            findWhiteSquares(i - 1, j, x, y, image, iteration + 1);
        }

        Vec3b jMinColor = image.at<Vec3b>(i, j - 1);
        if ( isWhite(jMinColor, g_whiteMargin - g_deltaWhiteMargin) ) {
            image.at<Vec3b>(i, j - 1) = green;
            findWhiteSquares(i, j - 1, x, y, image, iteration + 1);
        }

        Vec3b iPlusColor = image.at<Vec3b>(i + 1, j);
        if ( isWhite(iPlusColor, g_whiteMargin - g_deltaWhiteMargin) ) {
            image.at<Vec3b>(i + 1, j) = green;
            findWhiteSquares(i + 1, j, x, y, image, iteration + 1);
        }

        Vec3b jPlusColor = image.at<Vec3b>(i, j + 1);
        if ( isWhite(jPlusColor, g_whiteMargin - g_deltaWhiteMargin) ) {
            image.at<Vec3b>(i, j + 1) = green;
            findWhiteSquares(i, j + 1, x, y, image, iteration + 1);
        }
    }
}

bool ArucoMarkerfinder::findMarkerData(std::vector<int> &x, std::vector<int> &y, int sI, int eI,
                                       Mat image, std::vector<int> &markerData, std::vector<float> &posRot) {

    if ( eI-sI < (g_minMarkerPixels/( g_skipPixels*g_skipPixels )) ) {
        // not enough pixels to be an aruco marker
        return false;
    }
    int aIndex = sI, bIndex = sI, cIndex = 0, dIndex = 0, xCenter = 0, yCenter = 0;
    float xDst, yDst;
    float distance = 0;
    float distanceTest;

    // find pixel furthest away from your starting index, this is corner 'a'
    for (int I = sI; I < eI; I++) {
        xDst = x[I] - x[sI];
        yDst = y[I] - y[sI];
        distanceTest = (xDst*xDst + yDst*yDst);
        if ( distanceTest > distance ) {
            distance = distanceTest;
            aIndex = I;
        }
    }
    distance = 0;

    // find pixel furthest away from corner 'a', this is corner 'b'
    for (int I = sI; I < eI; I++) {
        xDst = x[I] - x[aIndex];
        yDst = y[I] - y[aIndex];
        distanceTest = (xDst*xDst + yDst*yDst);
        if ( distanceTest > distance ) {
            distance = distanceTest;
            bIndex = I;
        }
    }
    distance = 0;

    // find the center pixel and get a pixel with d = (.5(b-a)) and dot product 0, which should give corner 'c' as the furthest pixel
    float xabDst = x[aIndex] - x[bIndex];
    float yabDst = y[aIndex] - y[bIndex];
    xCenter = (int)round(0.5*xabDst + x[bIndex]);
    yCenter = (int)round(0.5*yabDst + y[bIndex]);

    auto xPointTest = (int)(xCenter - yabDst);
    auto yPointTest = (int)(yCenter + xabDst);

    // find the pixel furthest away from this random pixel, this is corner 'c'
    for (int I = sI; I < eI; I++) {
        xDst = x[I] - xPointTest;
        yDst = y[I] - yPointTest;
        distanceTest = (xDst*xDst + yDst*yDst);
        if ( distanceTest > distance ) {
            distance = distanceTest;
            cIndex = I;
        }
    }
    distance = 0;

    // find the pixel furthest away from corner 'c', this is corner 'd'
    for (int I = sI; I < eI; I++) {
        xDst = x[I] - x[cIndex];
        yDst = y[I] - y[cIndex];
        distanceTest = (xDst*xDst + yDst*yDst);
        if ( distanceTest > distance ) {
            distance = distanceTest;
            dIndex = I;
        }
    }

    // vector calculations for angle theta and the positions of the aruco data

    // vectors
    float wx = 0, wy = 1;
    float ux = x[aIndex] - x[cIndex], uy = y[aIndex] - y[cIndex];
    float vx = x[aIndex] - x[dIndex], vy = y[aIndex] - y[dIndex];
    float uux = x[dIndex] - x[bIndex], uuy = y[dIndex] - y[bIndex];
    float vvx = x[cIndex] - x[bIndex], vvy = y[cIndex] - y[bIndex];

    // dot products
    float uwDot = ux*wx + uy*wy;
    float uvDot = ux*vx + uy*vy;
    float uuvDot = uux*vx + uuy*vy;
    float uvvDot = ux*vvx + uy*vvy;
    float uuvvDot = uux*vvx + uuy*vvy;

    // lengths
    float wLength = sqrt(wx*wx + wy*wy);
    float uLength = sqrt(ux*ux + uy*uy);
    float vLength = sqrt(vx*vx + vy*vy);
    float uuLength = sqrt(uux*uux + uuy*uuy);
    float vvLength = sqrt(vvx*vvx + vvy*vvy);

    // angles
    float uwCosTheta = uwDot / (uLength * wLength);
    float uvCosTheta = ( uvDot / (uLength * vLength) );
    float uuvCosTheta = ( uuvDot / (uuLength * vLength) );
    float uvvCosTheta = ( uvvDot / (uLength * vvLength) );
    float uuvvCosTheta = ( uuvvDot / (uuLength * vvLength) );

    // test if we are dealing with a square
    float maxAcosAngle = 0.25;
    if (abs(uvCosTheta) > maxAcosAngle || abs(uuvCosTheta) > maxAcosAngle || abs(uvvCosTheta) > maxAcosAngle || abs(uuvvCosTheta) > maxAcosAngle) {
        // the angles are too far off 90 degrees (maxAcosAngle = magic number..)
        return false;
    }
    float deltaLength = 0.25;
    if ( abs(uLength - vLength)/uLength > deltaLength || abs(uLength - uuLength)/uLength > deltaLength || abs(uLength - vvLength)/uLength > deltaLength ) {
        // the differences in sidelengths is too large (deltaLength = magic number..)
        return false;
    }
    float theta = acos( uwCosTheta );

    // extract aruco data
    double xx;
    double yy;
    Vec3b green = {0,255,0};
    for (int i = 0; i < ARUCOSIZE; i++) {
        for (int j = 0; j < ARUCOSIZE; j++) {
            xx = x[aIndex] - ( (i+1.5)*( (ARUCOSIZE-j+0.5)*ux + (j+1.5)*uux ) + (j+1.5)*( (ARUCOSIZE-i+0.5)*vx + (i+1.5)*vvx ) )/( (ARUCOSIZE+2)*(ARUCOSIZE+2) );
            yy = y[aIndex] - ( (i+1.5)*( (ARUCOSIZE-j+0.5)*uy + (j+1.5)*uuy ) + (j+1.5)*( (ARUCOSIZE-i+0.5)*vy + (i+1.5)*vvy ) )/( (ARUCOSIZE+2)*(ARUCOSIZE+2) );
            Mat isWhite = Mat::zeros(1,1,CV_8UC3);

            Vec3b color = image.at<Vec3b>((int)round(xx), (int)round(yy));

            if ( color == green ) {
                image.at<Vec3b>((int)round(xx),(int)round(yy)) = {255,127,63};
                markerData.push_back(1);
            }
            else {
                image.at<Vec3b>((int)round(xx),(int)round(yy)) = {127,255,63};
                markerData.push_back(0);
            }

        }
    }

    // visualization of the corners/center in color red
    image.at<Vec3b>(x[aIndex],y[aIndex]) = {0,0,255};
    image.at<Vec3b>(x[bIndex],y[bIndex]) = {0,0,255};
    image.at<Vec3b>(x[cIndex],y[cIndex]) = {0,0,255};
    image.at<Vec3b>(x[dIndex],y[dIndex]) = {0,0,255};

    posRot.push_back(xCenter);
    posRot.push_back(yCenter);
    posRot.push_back(theta);

    return true;
}

void ArucoMarkerfinder::findMarkerId(std::vector<int> &resultData, std::vector<int> &markerIds) {


}

void ArucoMarkerfinder::findMarkers(Mat image, std::vector<int> &markerIds, std::vector<int> &markerX, std::vector<int> &markerY, std::vector<int> &markerTheta) {

    // create vector of all x- and y-positions of the white pixels

    std::vector<int> xSqMarker;
    std::vector<int> ySqMarker;
    std::vector<int> startIndSqMarker = {0};

    Vec3b green = {0,255,0};
    // iterate through all pixels, skipping g_skipPixels- pixels, and find white ones.
    for (int i = g_skipPixels+1; i < image.rows - g_skipPixels; i += g_skipPixels) {
        for (int j = g_skipPixels+1; j < image.cols - g_skipPixels; j += g_skipPixels) {
            auto &color = image.at<Vec3b>(i, j);
            if (color != green) {
                findWhitePixels(i, j, xSqMarker, ySqMarker, startIndSqMarker, image);

            }
        }
    }

    std::vector<int> xCorners;
    std::vector<int> yCorners;
    std::vector<int> markerData;
    std::vector<float> posRot;

    for (int marker = 0; marker < (int) startIndSqMarker.size() - 1; marker++) {
        int startInd = startIndSqMarker[marker];
        int endInd = startIndSqMarker[marker + 1];
        bool isMarker;
        isMarker = findMarkerData(xSqMarker, ySqMarker, startInd, endInd, image, markerData, posRot);
        if (isMarker) {
            findMarkerId(markerData, markerIds);
        }
    }
}





