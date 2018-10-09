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
bool ArucoMarkerfinder::findMarkerData(std::vector<int> &x, std::vector<int> &y, int sI, int eI, Mat image, std::vector<bool> &markerData, std::vector<int> &corners) {

    if ( eI-sI < g_minMarkerPixels ) {
        std::cerr << "marker dismissed: not enough pixels" << std::endl;
        return false;
    }
    std::vector<int> index = {sI, sI, 0, 0};
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
            index[0] = I;
        }
    }
    distance = 0;

    // find pixel furthest away from corner 'a', this is corner 'b'
    for (int I = sI; I < eI; I++) {
        xDst = x[I] - x[index[0]];
        yDst = y[I] - y[index[0]];
        distanceTest = (xDst*xDst + yDst*yDst);
        if ( distanceTest > distance ) {
            distance = distanceTest;
            index[1] = I;
        }
    }
    distance = 0;

    // find the center pixel and get a pixel with d = (.5(b-a)) and dot product 0, which should give corner 'c' as the furthest pixel
    float xabDst = x[index[0]] - x[index[1]];
    float yabDst = y[index[0]] - y[index[1]];
    xCenter = 0.5f*xabDst + x[index[1]];
    yCenter = 0.5f*yabDst + y[index[1]];

    auto xPointTest = (int)(xCenter - yabDst);
    auto yPointTest = (int)(yCenter + xabDst);

    // find the pixel furthest away from this calculated pixel, this is corner 'c'
    for (int I = sI; I < eI; I++) {
        xDst = x[I] - xPointTest;
        yDst = y[I] - yPointTest;
        distanceTest = (xDst*xDst + yDst*yDst);
        if ( distanceTest > distance ) {
            distance = distanceTest;
            index[2] = I;
        }
    }
    distance = 0;

    // find the pixel furthest away from corner 'c', this is corner 'd'
    for (int I = sI; I < eI; I++) {
        xDst = x[I] - x[index[2]];
        yDst = y[I] - y[index[2]];
        distanceTest = (xDst*xDst + yDst*yDst);
        if ( distanceTest > distance ) {
            distance = distanceTest;
            index[3] = I;
        }
    }

    // vector calculations for angle theta and the positions of the aruco data
    // vector w is the vertical vector to compare the angle to

    // vectors of the square sides
    float ux = x[index[0]] - x[index[2]], uy = y[index[0]] - y[index[2]];
    float vx = x[index[0]] - x[index[3]], vy = y[index[0]] - y[index[3]];
    float uux = x[index[3]] - x[index[1]], uuy = y[index[3]] - y[index[1]];
    float vvx = x[index[2]] - x[index[1]], vvy = y[index[2]] - y[index[1]];

    // dot products between vectors
    float uvDot = ux*vx + uy*vy;
    float uuvDot = uux*vx + uuy*vy;
    float uvvDot = ux*vvx + uy*vvy;
    float uuvvDot = uux*vvx + uuy*vvy;

    // sidelengths of the square
    float uLength = sqrt(ux*ux + uy*uy);
    float vLength = sqrt(vx*vx + vy*vy);
    float uuLength = sqrt(uux*uux + uuy*uuy);
    float vvLength = sqrt(vvx*vvx + vvy*vvy);

    // cosine of the angles
    float uvCosTheta = ( uvDot / (uLength * vLength) );
    float uuvCosTheta = ( uuvDot / (uuLength * vLength) );
    float uvvCosTheta = ( uvvDot / (uLength * vvLength) );
    float uuvvCosTheta = ( uuvvDot / (uuLength * vvLength) );

    // test if we are dealing with (close to) a square
    float maxAcosAngle = 0.25;
    if (abs(uvCosTheta) > maxAcosAngle || abs(uuvCosTheta) > maxAcosAngle || abs(uvvCosTheta) > maxAcosAngle || abs(uuvvCosTheta) > maxAcosAngle) {
        std::cerr << "marker dismissed: the marker is not square (enough)" << std::endl;
        return false;
    }
    float deltaLength = 0.25;
    if ( abs(uLength - vLength)/uLength > deltaLength || abs(uLength - uuLength)/uLength > deltaLength || abs(uLength - vvLength)/uLength > deltaLength ) {
        std::cerr << "marker dismissed: the marker is not square (enough)" << std::endl;
        return false;
    }

    // send back the corner data
    for (int i = 0; i < 4; i++) {
        corners.push_back(x[index[i]]);
        corners.push_back(y[index[i]]);
        // visualization of the corners
        image.at<Vec3b>(x[index[i]],y[index[i]]) = {0,0,255};

    }

    // extract marker bit data
    double xx;
    double yy;
    Vec3b green = {0,255,0};
    int edgeCheck = 0;
    for (int i = -1; i <= ARUCOSIZE; i++) {
        for (int j = -1; j <= ARUCOSIZE; j++) {
            xx = x[index[0]] - ( (j+1.5)*( (ARUCOSIZE-i+0.5)*ux + (i+1.5)*uux ) + (i+1.5)*( (ARUCOSIZE-j+0.5)*vx + (j+1.5)*vvx ) )/( (ARUCOSIZE+2)*(ARUCOSIZE+2) );
            yy = y[index[0]] - ( (j+1.5)*( (ARUCOSIZE-i+0.5)*uy + (i+1.5)*uuy ) + (i+1.5)*( (ARUCOSIZE-j+0.5)*vy + (j+1.5)*vvy ) )/( (ARUCOSIZE+2)*(ARUCOSIZE+2) );

            auto &color = image.at<Vec3b>((int)round(xx), (int)round(yy));

            if (i == -1 || j == -1 || i == ARUCOSIZE || j == ARUCOSIZE) {
                // at the edges, check if (most) pixels are white. if not, dismiss marker.
                if (color == green) {
                    image.at<Vec3b>((int)round(xx),(int)round(yy)) = {0,220,0};

                } else {
                    edgeCheck++;
                    if (edgeCheck > 2) {
                        std::cerr << "marker dismissed: the edge is not fully white" << std::endl;
                        return false;
                    }

                }
            } else if ( color == green ) {
                // in the center data structure, save the data of the bits
                image.at<Vec3b>((int)round(xx),(int)round(yy)) = {255,127,63};
                markerData.push_back(false);
            }
            else {
                image.at<Vec3b>((int)round(xx),(int)round(yy)) = {127,255,63};
                markerData.push_back(true);
            }
        }
    }

    return true;
}

/// finds robot id assigned to the aruco marker data
bool ArucoMarkerfinder::findMarkerId(std::vector<bool> &resultData, std::vector<float> &posRotID, std::vector<int> &corners) {
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
    std::vector<float> orientation;

    bool one = resultData[1], three = resultData[3], five = resultData[5], seven = resultData[7];
    // case 1:
    if (one && !seven && ((three && five) || (!three && !five))) {

        orientation = {0,3,2,1};     // variables for determing the angle

        for (int i = 0; i < 5; i++) {
            // D = 6,0,4,8,2
            int D = (6 + i * 4) % 10;
            if (resultData[D]) {
                id += (int) pow(2, i);
                parity++;
            }
        }
        if ((parity % 2 == 1 && (three && five)) || (parity % 2 == 0 && (!three && !five))) {
            std::cerr << "marker dismissed: parity bit incorrect" << std::endl;
            return false;
        }
    }
        // case 3:
    else if (three && !five && ((one && seven) || (!one && !seven))) {

        orientation = {0,2,3,1};

        for (int i = 0; i < 5; i++) {
            // D = 8,6,4,2,0
            int D = 8 - 2 * i;
            if (resultData[D]) {
                id += (int) pow(2, i);
                parity++;
            }
        }
        if ((parity % 2 == 1 && (three && five)) || (parity % 2 == 0 && (!three && !five))) {
            std::cerr << "marker dismissed: parity bit incorrect" << std::endl;
            return false;
        }
    }
        // case 5:
    else if (five && !three && ((one && seven) || (!one && !seven))) {

        orientation = {1,3,2,0};

        for (int i = 0; i < 5; i++) {
            // D = 0,2,4,6,8
            int D = 2 * i;
            if (resultData[D]) {
                id += (int) pow(2, i);
                parity++;
            }
        }
        if ((parity % 2 == 1 && (three && five)) || (parity % 2 == 0 && (!three && !five))) {
            std::cerr << "marker dismissed: parity bit incorrect" << std::endl;
            return false;
        }
    }
        // case 7:
    else if (seven && !one && ((three && five) || (!three && !five))) {

        orientation = {1,2,3,0};

        for (int i = 0; i < 5; i++) {
            // D = 2,8,4,0,6
            int D = (2 + i * 6) % 10;
            if (resultData[D]) {
                id += (int) pow(2, i);
                parity++;
            }
        }
        // checking the parity bits: n(D) odd -> P should be 0, n(D) even -> P should be 1
        if ((parity % 2 == 1 && (three && five)) || (parity % 2 == 0 && (!three && !five))) {
            std::cerr << "marker dismissed: parity bit incorrect" << std::endl;
            return false;
        }

    } else {    // else it is not a marker in our dictionary
        std::cerr << "marker dismissed: parity/direction combination incorrect" << std::endl;
        return false;
    }

    posRotID.push_back(id);

    auto ax = (float)corners[0], bx = (float)corners[2], cx = (float)corners[4], dx = (float)corners[6];
    auto ay = (float)corners[1], by = (float)corners[3], cy = (float)corners[5], dy = (float)corners[7];

    // the id is extracted from the data, now we calculate the position of the center and the rotation of the marker

    // calculate the center of the square using vectors:
    //     a ____ d         vector p = a + t(b-a);      vector q = c + s(d-c);
    //      |    |
    //     c|____|b     ->  center of the square is the intersection between the two vectors.
    //
    //                  s =     ( (ay-cy)(bx-ax) + (cx-ax)(by-ay) ) / ( (dy-cy)(bx-ax) + (cx-dx)(by-ay) )
    //            xCenter =     cx + s(dx - cx)
    //            yCenter =     cy + s(dy - cy)

    if ((dy - cy) * (bx - ax) + (cx - dx) * (by - ay) != 0) {
        float s = (((ay - cy) * (bx - ax) + (cx - ax) * (by - ay)) /
                   ((dy - cy) * (bx - ax) + (cx - dx) * (by - ay)));
        float xCenter = cx + s * (dx - cx);
        float yCenter = cy + s * (dy - cy);

        posRotID.push_back(xCenter);
        posRotID.push_back(yCenter);

    } else {
        std::cerr << "marker dismissed, divide by zero??: " << (dy - cy) * (bx - ax) + (cx - dx) * (by - ay)
                  << std::endl;
        return false;
    }

    //      the rotation of the marker is calculated by taking the two corners closest to the direction "1" bit
    //      the direction is calculated by the average average vector of the two corners closest to the "1" bit
    //          and the corners furthest away from this bit.

    float xVector;
    float yVector;
    std::vector<float> thetaVector;

    // using the two points closest to the "1" bit (orientation vector), calculate the direction
    for (int i = 0; i < 2; i++) {
        xVector = corners[(int)(2 * orientation[2 * i])] - corners[(int)(2 * orientation[2 * i + 1])];
        yVector = corners[(int)(2 * orientation[2 * i] + 1)] - corners[(int)(2 * orientation[2 * i + 1] + 1)];

        if (xVector > 0) thetaVector.push_back(atan(yVector / xVector));
        else if (xVector < 0) thetaVector.push_back((float)(CV_PI + atan(yVector / xVector)));
        else thetaVector.push_back((float) (CV_PI * 0.5));

    }
    auto angle = (float)( (thetaVector[0]+thetaVector[1]) * 0.5 );
    if (angle > 1.0) angle -= 2*CV_PI;
    posRotID.push_back( angle );

    return true;
}

/// finds the robot id, x,y position and rotations of all robots
void ArucoMarkerfinder::findMarkers(Mat image, std::vector<int> &markerID, std::vector<int> &markerX, std::vector<int> &markerY, std::vector<float> &markerTheta) {

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

    // for all square markers found
    for (int marker = 0; marker < (int) startIndSqMarker.size() - 1; marker++) {
        // check if the thing we are dealing with is a square and extract the marker bit data
        int startInd = startIndSqMarker[marker];
        int endInd = startIndSqMarker[marker + 1];
        bool isMarker;
        bool isRobotID;
        std::vector<int> corners;
        std::vector<float> posRotID;
        isMarker = findMarkerData(xSqMarker, ySqMarker, startInd, endInd, image, markerData, corners);

        // if we are dealing with a marker, connect the marker bit data with the robot id
        if (isMarker) {

            isRobotID = findMarkerId(markerData, posRotID, corners);
            if (isRobotID) {

                auto id = (int)round(posRotID[0]);
                auto x = (int)round(posRotID[1]);
                auto y = (int)round(posRotID[2]);
                float theta = posRotID[3];

                markerID.push_back(id);
                markerX.push_back(x);
                markerY.push_back(y);
                markerTheta.push_back(theta);
            }
        }
        markerData.clear();
        posRotID.clear();
        corners.clear();
    }
}

