//
// Created by thijs on 2-10-18.
//

#include "aruco_markerfinder.h"

ArucoMarkerfinder::ArucoMarkerfinder() = default;

/// checks fotr a pixel if it is white
bool ArucoMarkerfinder::isWhite(Vec3b &color, int white) {
    // assume a pixel is white, and disprove it.
    bool pixelIsWhite = true;
    for (int c = 0; c <= 2; c++) {
        if (color[c] < white) {
            pixelIsWhite = false;
        }
    }
    return pixelIsWhite;
}

/// checks if a pixel is white. if it is, find pixels around it and make a blob of white
void ArucoMarkerfinder::findWhitePixels(int i, int j, std::vector<int> &x, std::vector<int> &y, std::vector<int> &index, Mat image) {

    auto &color = image.at<Vec3b>(i,j);
    if ( isWhite(color, g_whiteMargin) ) {
        // if a white pixels is found, create a grouped blo of pixels
        findWhiteSquares(i,j,x,y,image,0);
        index.push_back((int) x.size());
    }
}

/// recursively groups white pixels
void ArucoMarkerfinder::findWhiteSquares(int i, int j, std::vector<int> &x, std::vector<int> &y, Mat image, int iteration) {
    // push the current pixel data for this blob/square into a vector
    x.push_back(i);
    y.push_back(j);

    Vec3b green = {0,255,0};

    if (iteration == 0) {
        // set the first pixel in the square to grey
        Vec3b grey = {70,60,70};
        image.at<Vec3b>(i,j) = grey;
    }

    // if whe are not at the edge.. check left,up,right,down for pixels in the current square
    if (iteration < MAXRECURSION && (i > 1) && (j > 1) && (i < image.rows-1) && (j < image.cols-1) ) {

        // if the pixel at i-1,j is white, mark it with color green and check the squares around that pixel
        auto &iMinColor = image.at<Vec3b>(i - 1, j);
        if ( isWhite(iMinColor, g_whiteMargin - g_deltaWhiteMargin) ) {
            image.at<Vec3b>(i - 1, j) = green;
            findWhiteSquares(i - 1, j, x, y, image, iteration + 1);
        }

        auto &jMinColor = image.at<Vec3b>(i, j - 1);
        if ( isWhite(jMinColor, g_whiteMargin - g_deltaWhiteMargin) ) {
            image.at<Vec3b>(i, j - 1) = green;
            findWhiteSquares(i, j - 1, x, y, image, iteration + 1);
        }

        auto &iPlusColor = image.at<Vec3b>(i + 1, j);
        if ( isWhite(iPlusColor, g_whiteMargin - g_deltaWhiteMargin) ) {
            image.at<Vec3b>(i + 1, j) = green;
            findWhiteSquares(i + 1, j, x, y, image, iteration + 1);
        }

        auto &jPlusColor = image.at<Vec3b>(i, j + 1);
        if ( isWhite(jPlusColor, g_whiteMargin - g_deltaWhiteMargin) ) {
            image.at<Vec3b>(i, j + 1) = green;
            findWhiteSquares(i, j + 1, x, y, image, iteration + 1);
        }
    }
}

/// for each group of pixels, check if it actually is an aruco marker
bool ArucoMarkerfinder::findMarkerData(std::vector<int> &x, std::vector<int> &y, int sI, int eI, Mat image, std::vector<bool> &markerData, std::vector<float> &posRot) {

    if ( eI-sI < (g_minMarkerPixels/( g_skipPixels*g_skipPixels )) ) {
        // not enough pixels to be an aruco marker, or to determine the data accurately
        return false;
    }

    int aIndex = sI, bIndex = sI, cIndex = 0, dIndex = 0;
    float xCenter = 0, yCenter = 0;
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
    xCenter = 0.5f*xabDst + x[bIndex];
    yCenter = 0.5f*yabDst + y[bIndex];

    auto xPointTest = (int)(xCenter - yabDst);
    auto yPointTest = (int)(yCenter + xabDst);

    // find the pixel furthest away from this calculated pixel, this is corner 'c'
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
    // vector w is the vertical vector to compare the angle to

    // vectors of the square sides
    float wx = 0, wy = 1;

    float ux = x[aIndex] - x[cIndex], uy = y[aIndex] - y[cIndex];
    float vx = x[aIndex] - x[dIndex], vy = y[aIndex] - y[dIndex];
    float uux = x[dIndex] - x[bIndex], uuy = y[dIndex] - y[bIndex];
    float vvx = x[cIndex] - x[bIndex], vvy = y[cIndex] - y[bIndex];

    // dot products between vectors
    float uwDot = ux*wx + uy*wy;
    float uuwDot = uux*wx + uuy*wy;
    float vwDot = vx*wx + vy*wy;
    float vvwDot = vvx*wx + vvy*wy;

    float uvDot = ux*vx + uy*vy;
    float uuvDot = uux*vx + uuy*vy;
    float uvvDot = ux*vvx + uy*vvy;
    float uuvvDot = uux*vvx + uuy*vvy;

    // sidelengths of the square
    float wLength = sqrt(wx*wx + wy*wy);

    float uLength = sqrt(ux*ux + uy*uy);
    float vLength = sqrt(vx*vx + vy*vy);
    float uuLength = sqrt(uux*uux + uuy*uuy);
    float vvLength = sqrt(vvx*vvx + vvy*vvy);

    // cosine of the angles
    float uwCosTheta = uwDot / (uLength * wLength);
    float uuwCosTheta = uuwDot / (uuLength * wLength);
    float vwCosTheta = vwDot / (vLength * wLength);
    float vvwCosTheta = vvwDot / (vvLength * wLength);

    float uvCosTheta = ( uvDot / (uLength * vLength) );
    float uuvCosTheta = ( uuvDot / (uuLength * vLength) );
    float uvvCosTheta = ( uvvDot / (uLength * vvLength) );
    float uuvvCosTheta = ( uuvvDot / (uuLength * vvLength) );

    // test if we are dealing with (close to) a square
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
    float uTheta = acos( uwCosTheta );
    float uuTheta = acos( uuwCosTheta );
    float vTheta = acos( vwCosTheta );
    float vvTheta = acos( vvwCosTheta );
    std::cout << "angles: " << "" << uTheta << " x " << uuTheta << " x " << vTheta << " x " << vvTheta << " x " << std::endl;

    // extract marker bit data
    double xx;
    double yy;
    Vec3b green = {0,255,0};
    for (int j = 0; j < ARUCOSIZE; j++) {
        for (int i = 0; i < ARUCOSIZE; i++) {
            xx = x[aIndex] - ( (i+1.5)*( (ARUCOSIZE-j+0.5)*ux + (j+1.5)*uux ) + (j+1.5)*( (ARUCOSIZE-i+0.5)*vx + (i+1.5)*vvx ) )/( (ARUCOSIZE+2)*(ARUCOSIZE+2) );
            yy = y[aIndex] - ( (i+1.5)*( (ARUCOSIZE-j+0.5)*uy + (j+1.5)*uuy ) + (j+1.5)*( (ARUCOSIZE-i+0.5)*vy + (i+1.5)*vvy ) )/( (ARUCOSIZE+2)*(ARUCOSIZE+2) );
            Mat isWhite = Mat::zeros(1,1,CV_8UC3);

            auto &color = image.at<Vec3b>((int)round(xx), (int)round(yy));

            if ( color == green ) {
                image.at<Vec3b>((int)round(xx),(int)round(yy)) = {255,127,63};
                markerData.push_back(false);
            }
            else {
                image.at<Vec3b>((int)round(xx),(int)round(yy)) = {127,255,63};
                markerData.push_back(true);
            }

        }
    }

    // visualization of the corners of the square in color red
    image.at<Vec3b>(x[aIndex],y[aIndex]) = {0,0,255};
    image.at<Vec3b>(x[bIndex],y[bIndex]) = {0,0,255};
    image.at<Vec3b>(x[cIndex],y[cIndex]) = {0,0,255};
    image.at<Vec3b>(x[dIndex],y[dIndex]) = {0,0,255};

    // send back the data
    posRot.push_back(xCenter);
    posRot.push_back(yCenter);

    // TODO: find the actual angle theta using the 4 (slightly different) angles of the square
    posRot.push_back(uTheta);

    return true;
}

/// finds robot id assigned to the aruco marker data
bool ArucoMarkerfinder::findMarkerId(std::vector<bool> &resultData, std::vector<int> &markerIds) {
    //
    //      resultData is stored in the order as seen in the figure below                                           |
    //      _________       _________                                                                               |
    //                                      D = data                                                                |
    //      | 0 1 2 |       | D 1 D |       0 = always white                                                        |
    //      | 3 4 5 |       | P D P |       1 = always black                                                        |
    //      | 6 7 8 |       | D 0 D |       P = parity:                                                             |
    //      _________       _________       "n(D) even -> P = 1" | "n(D) odd -> P = 0"                              |
    //                                                                                                              |
    //      rotation of the marker is unknown, so the "1" pixel can be in spot 2, 4, 6 or 8                         |
    //      in the upright position (case 1), data is read in the following order:                                  |
    //                                  ____________________________________________________________________________|
    //      case 1: 6,0,4,8,2           | example:  | id = 24, rotation = "case 3" -> resultData[3] = 1             |
    //      case 3: 8,6,4,2,0           |           | 24(binary) = 11000 -> resultData[0] = 1, resultData[2] = 1    |
    //      case 5: 0,2,4,6,8           |           | n(D) even -> P = 1 -> resultData[7] = 1, resultData[1] = 1    |
    //      case 7: 2,8,4,0,6           |           | }-> [0,1,2,3,7] = "1", [4,5,6,8] = "0"                        |
    //

    int id = 0;
    int parity = 0;
    bool one = resultData[1], three = resultData[3], five = resultData[5], seven = resultData[7];
    // case 1:
    if (one && !seven && ( (three && five) || (!three && !five) )) {
        for (int i = 0; i < 5; i++) {
            // D = 6,0,4,8,2
            int D = (6+i*4) % 10;
            if (resultData[D]) {
                id += (int)pow(2, i);
                parity++;
            }
        }
        if ( (parity % 2 == 1 && (three && five) ) || (parity % 2 == 0 && (!three && !five) ) ) {
            std::cerr << "marker dismissed: parity bit incorrect" << std::endl;
            return false;
        }
    }
        // case 3:
    else if (three && !five && ( (one && seven) || (!one && !seven) )) {
        for (int i = 0; i < 5; i++) {
            // D = 8,6,4,2,0
            int D = 8-2*i;
            if (resultData[D]) {
                id += (int)pow(2, i);
                parity++;
            }
        }
        if ( (parity % 2 == 1 && (three && five) ) || (parity % 2 == 0 && (!three && !five) ) ) {
            std::cerr << "marker dismissed: parity bit incorrect" << std::endl;
            return false;
        }
    }
        // case 5:
    else if (five && !three && ( (one && seven) || (!one && !seven) )) {
        for (int i = 0; i < 5; i++) {
            // D = 0,2,4,6,8
            int D = 2*i;
            if (resultData[D]) {
                id += (int)pow(2, i);
                parity++;
            }
        }
        if ( (parity % 2 == 1 && (three && five) ) || (parity % 2 == 0 && (!three && !five) ) ) {
            std::cerr << "marker dismissed: parity bit incorrect" << std::endl;
            return false;
        }
    }
        // case 7:
    else if (seven && !one && ( (three && five) || (!three && !five) )) {
        for (int i = 0; i < 5; i++) {
            // D = 2,8,4,0,6
            int D = (2+i*6) % 10;
            if (resultData[D]) {
                id += (int)pow(2, i);
                parity++;
            }
        }
        if ( (parity % 2 == 1 && (three && five) ) || (parity % 2 == 0 && (!three && !five) ) ) {
            std::cerr << "marker dismissed: parity bit incorrect" << std::endl;
            return false;
        }
    }
        // else it is not a valid marker
    else {
        std::cerr << "marker dismissed: parity/direction combination incorrect" << std::endl;
        return false;
    }

    markerIds.push_back(id);
    return true;
}

/// finds the robot id, x,y position and rotations of all robots
void ArucoMarkerfinder::findMarkers(Mat image, std::vector<int> &markerIds, std::vector<int> &markerX, std::vector<int> &markerY, std::vector<float> &markerTheta) {

    // create vector of all x- and y-positions of the white pixels
    std::vector<int> xSqMarker;
    std::vector<int> ySqMarker;
    std::vector<int> startIndSqMarker = {0};

    // iterate through all pixels, skipping g_skipPixels- pixels, and find white ones.
    Vec3b green = {0,255,0};
    for (int i = g_skipPixels+1; i < image.rows - g_skipPixels; i += g_skipPixels) {
        for (int j = g_skipPixels+1; j < image.cols - g_skipPixels; j += g_skipPixels) {
            // if the pixel is not yet part of a square marker
            auto &color = image.at<Vec3b>(i, j);
            if (color != green) {
                findWhitePixels(i, j, xSqMarker, ySqMarker, startIndSqMarker, image);

            }
        }
    }

    // create vector for the data to send back
    std::vector<bool> markerData;
    std::vector<float> posRot;

    // for all square markers found
    for (int marker = 0; marker < (int) startIndSqMarker.size() - 1; marker++) {
        // check if the thing we are dealing with is a square and extract the marker bit data
        int startInd = startIndSqMarker[marker];
        int endInd = startIndSqMarker[marker + 1];
        bool isMarker;
        bool isRobotID = false;
        isMarker = findMarkerData(xSqMarker, ySqMarker, startInd, endInd, image, markerData, posRot);

        // if we are dealing with a marker, connect the marker bit data with the robot id
        if (isMarker) {
            isRobotID = findMarkerId(markerData, markerIds);
            if (isRobotID) {
                markerX.push_back((int)posRot[0]);
                markerY.push_back((int)posRot[1]);
                markerTheta.push_back(posRot[2]);
            }
        }
        markerData.clear();
        posRot.clear();
    }
}

