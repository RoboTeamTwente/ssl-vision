//
// Created by thijs on 2-10-18.
//

#include "aruco_markerfinder.h"

ArucoMarkerfinder::ArucoMarkerfinder() = default;

void ArucoMarkerfinder::findWhitePixels(int i, int j, std::vector<int> &x, std::vector<int> &y, Vec3b color, Mat result) {
    // assume the pixel is white, unless proven otherwise.. if white -> put data in new matrix.
    bool pixelIsWhite = true;
    char twoPixelsWhite = 2;
    for (int c = 0; c <= 2; c++) {
        if (color[c] < g_whiteMargin) {
            pixelIsWhite = false;
            twoPixelsWhite--;
        }
    }
    if (pixelIsWhite || twoPixelsWhite >= 1) {
        result.at<Vec3b>(i/g_skipPixels,j/g_skipPixels) = {0,255,0};
        x.push_back(i);
        y.push_back(j);
    }


}

void ArucoMarkerfinder::findWhiteSquares(int i, int j, std::vector<int> &x, std::vector<int> &y, Mat whitePixels, int iteration) {
    // push the current pixel data for this blob/square into a vector
    x.push_back(i);
    y.push_back(j);

    // 'itColorFactor' and 'd' is used for visualization of the recursive pixel checking
    unsigned char itColorFactor = 1;
    Vec3b green = {0,255,0};
    if (iteration == 0) {
        // set the first pixel in the square to white
        Vec3b newColor = {255,255,255};
        whitePixels.at<Vec3b>(i-1,j) = newColor;
    }

    // if whe are not at the edge.. check left,up,right,down for pixels in the current square
    if (iteration < MAXRECURSION && (i > 1) && (j > 1) && (i < whitePixels.rows-1) && (j < whitePixels.cols-1) ) {
        if (whitePixels.at<Vec3b>(i - 1, j) == green) {
            auto d = (unsigned char)( iteration * itColorFactor );
            Vec3b newColor = {d, (unsigned char) (255 - d), d};
            whitePixels.at<Vec3b>(i - 1, j) = newColor;
            findWhiteSquares(i - 1, j, x, y, whitePixels, iteration + 1);
        }
        if (whitePixels.at<Vec3b>(i, j - 1) == green) {
            auto d = (unsigned char)( iteration * itColorFactor );
            Vec3b newColor = {d, (unsigned char) (255 - d), d};
            whitePixels.at<Vec3b>(i, j - 1) = newColor;
            findWhiteSquares(i, j - 1, x, y, whitePixels, iteration + 1);
        }
        if (whitePixels.at<Vec3b>(i + 1, j) == green) {
            auto d = (unsigned char)( iteration * itColorFactor );
            Vec3b newColor = {d, (unsigned char) (255 - d), d};
            whitePixels.at<Vec3b>(i + 1, j) = newColor;
            findWhiteSquares(i + 1, j, x, y, whitePixels, iteration + 1);
        }
        if (whitePixels.at<Vec3b>(i, j + 1) == green) {
            auto d = (unsigned char)( iteration * itColorFactor );
            Vec3b newColor = {d, (unsigned char) (255 - d), d};
            whitePixels.at<Vec3b>(i, j + 1) = newColor;
            findWhiteSquares(i, j + 1, x, y, whitePixels, iteration + 1);
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

    // visualization of the corners/center in color red
    image.at<Vec3b>(x[aIndex]*g_skipPixels,y[aIndex]*g_skipPixels) = {0,0,255};
    image.at<Vec3b>(x[bIndex]*g_skipPixels,y[bIndex]*g_skipPixels) = {0,0,255};
    image.at<Vec3b>(x[cIndex]*g_skipPixels,y[cIndex]*g_skipPixels) = {0,0,255};
    image.at<Vec3b>(x[dIndex]*g_skipPixels,y[dIndex]*g_skipPixels) = {0,0,255};
    image.at<Vec3b>(xCenter*g_skipPixels,yCenter*g_skipPixels) = {0,0,255};

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
            std::vector<int> nothing;
            Vec3b color = image.at<Vec3b>((int)xx*g_skipPixels, (int)yy*g_skipPixels);
            findWhitePixels(1, 1, nothing, nothing, color, isWhite);
            if ( isWhite.at<Vec3b>(0,0) == green) {
                image.at<Vec3b>((int)round(xx)*g_skipPixels,(int)round(yy)*g_skipPixels) = {0,255,255};
                markerData.push_back(1);
            }
            else {
                image.at<Vec3b>((int)round(xx)*g_skipPixels,(int)round(yy)*g_skipPixels) = {255,255,0};
                markerData.push_back(0);
            }

        }
    }
    posRot.push_back(xCenter);
    posRot.push_back(yCenter);
    posRot.push_back(theta);

    return true;
}

void ArucoMarkerfinder::findMarkerId(std::vector<int> &resultData, std::vector<int> &markerIds) {


}

void ArucoMarkerfinder::findMarkers(Mat image, std::vector<int> &markerIds, std::vector<int> &markerX, std::vector<int> &markerY, std::vector<int> &markerTheta) {

    Mat resultWhitePixels;
    resultWhitePixels = Mat::zeros(image.rows / g_skipPixels, image.cols / g_skipPixels, CV_8UC3);

    // create vector of all x- and y-positions of the white pixels
    std::vector<int> xWhitePixels;
    std::vector<int> yWhitePixels;

    // iterate through all pixels, skipping g_skipPixels- pixels, and find white ones.
    for (int i = g_skipPixels+1; i < image.rows - g_skipPixels; i += g_skipPixels) {
        for (int j = g_skipPixels+1; j < image.cols - g_skipPixels; j += g_skipPixels) {
            auto &color = image.at<Vec3b>(i, j);
            findWhitePixels(i, j, xWhitePixels, yWhitePixels, color, resultWhitePixels);
        }
    }

    std::vector<int> xSqMarker;
    //xSqMarker.reserve(image.rows*image.cols+1);
    std::vector<int> ySqMarker;
    //xSqMarker.reserve(image.rows*image.cols+1);
    std::vector<int> startIndSqMarker = {0};

    for (unsigned int pix = 0; pix < xWhitePixels.size(); pix++) {

        int x = xWhitePixels.at(pix) / g_skipPixels;
        int y = yWhitePixels.at(pix) / g_skipPixels;
        Vec3b green = {0, 255, 0};
        if (resultWhitePixels.at<Vec3b>(x, y) == green) {
            findWhiteSquares(x, y, xSqMarker, ySqMarker, resultWhitePixels, 0);
            startIndSqMarker.push_back((int) xSqMarker.size());
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





