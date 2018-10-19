
//
// Created by Thijs Luttikhuis on 2-10-18.
//

#include "aruco_markerfinder.h"
//#define DEBUG

void ArucoMarkerfinder::setG_LowerWhiteMargin(const Vec3b &g_LowerWhiteMargin) {
    ArucoMarkerfinder::g_lowerWhiteMargin = g_LowerWhiteMargin;
}

void ArucoMarkerfinder::setG_upperWhiteMargin(const Vec3b &g_upperWhiteMargin) {
    ArucoMarkerfinder::g_upperWhiteMargin = g_upperWhiteMargin;
}

void ArucoMarkerfinder::setG_deltaWhiteMargin(const Vec3b &g_deltaWhiteMargin) {
    ArucoMarkerfinder::g_deltaWhiteMargin = g_deltaWhiteMargin;
}

/// Checks if a color is within certain bounds, input: (color, lower bound, upper bound)
bool ArucoMarkerfinder::isColor(Vec3b color, Vec3b lowerBound, Vec3b upperBound) {
    // Assume a pixel is white, and check if it is that color for all three color channels
    bool isColor = true;
    for (int c = 0; c < 3; c ++) {
        if (color[c] < lowerBound[c] || color[c] > upperBound[c]) {
            isColor = false;
        }
    }
    return isColor;
}
//

/// Finds the furthest pixel from vector 'relativeToPixel', looking in the selected indexes in vectors x & y
int ArucoMarkerfinder::findFurthestPixel(int xRelative, int yRelative, int startIndex, int endIndex,
                                         std::vector<int> &x, std::vector<int> &y) {

    int xDst;
    int yDst;
    int furthestIndex = startIndex;
    int distanceTest;
    int distance = 0;

    // for all indexes in the vector x and y between indexes startIndex and endIndex, check if it is further away from
    // x- and yRelative compared to the current furthest distance
    // If it is the furthest, this becomes the new furthest distance
    for (int I = startIndex; I < endIndex; I ++) {
        xDst = x[I] - xRelative;
        yDst = y[I] - yRelative;
        distanceTest = (xDst*xDst + yDst*yDst);
        if (distanceTest > distance) {
            distance = distanceTest;
            furthestIndex = I;
        }
    }
    return furthestIndex;
}

/// Finds the opposite corner of a (presumably) square blob of pixels
void ArucoMarkerfinder::findOppositeCorner(int xRelative, int yRelative, int startIndex, int endIndex,
                                           std::vector<int> &x, std::vector<int> &y, Vec3b &lowerBound, Vec3b &upperBound, std::vector<int> &corners,
                                           Mat image, Vec3b &setColor) {

    // using delta white margin, first find the opposite corner of a square
    // using that furthest corner, try to find more potential pixels as corner, finding pixels using a wider 'white' range
    // and find the opposite corner again using this new range
    int furthestIndex = findFurthestPixel(xRelative, yRelative, startIndex, endIndex, x, y);

    if (g_deltaWhiteMargin[0] == 0) {
        corners.push_back(x[furthestIndex]);
        corners.push_back(y[furthestIndex]);
        return;
    }
    // find a few new pixels with a wider color-margin in the corner
    std::vector<int> xCandidatePixels = {x[furthestIndex]}, yCandidatePixels = {y[furthestIndex]};
    groupWhitePixels(x[furthestIndex], y[furthestIndex], xCandidatePixels, yCandidatePixels, lowerBound, upperBound,
                     std::move(image), setColor);

    // find the furthest corner again using these new pixels
    auto candidateMaxIndex = (int) (xCandidatePixels.size() - 1);
    furthestIndex = findFurthestPixel(xRelative, yRelative, 0, candidateMaxIndex, xCandidatePixels, yCandidatePixels);

    corners.push_back(xCandidatePixels[furthestIndex]);
    corners.push_back(yCandidatePixels[furthestIndex]);

}

/// Creates four vectors of the sides of a square
void ArucoMarkerfinder::createVectors(std::vector<int> &x, std::vector<int> &y, std::vector<int> &index,
                                      std::vector<int> &v0, std::vector<int> &v1, std::vector<int> &v2,
                                      std::vector<int> &v3) {

    v0 = {index[0] - index[4], index[1] - index[5]};
    v1 = {index[0] - index[6], index[1] - index[7]};
    v2 = {index[6] - index[2], index[7] - index[3]};
    v3 = {index[4] - index[2], index[5] - index[3]};
}

/// Calculate the dot product between two vectors
int ArucoMarkerfinder::calcDotProduct(std::vector<int> &v1, std::vector<int> &v2) {
    return (v1[0]*v2[0] + v1[1]*v2[1]);
}

/// Calculate the length of a vector
float ArucoMarkerfinder::calcVectorLength(std::vector<int> &v1) {
    return (float)sqrt(v1[0] * v1[0] + v1[1] * v1[1]);
}

/// Calculate the cosine of the angle between two vectors
float ArucoMarkerfinder::calcCosAngle(float dot, float v0Length, float v1Length) {
    return dot/(v0Length*v1Length);
}

bool ArucoMarkerfinder::isSquareMarker(std::vector<int> &u, std::vector<int> &v, std::vector<int> &uu,
                                       std::vector<int> &vv) {
    // dot products between vectors
    int uvDot = calcDotProduct(u, v);
    int uuvDot = calcDotProduct(uu, v);
    int uvvDot = calcDotProduct(u, vv);
    int uuvvDot = calcDotProduct(uu, vv);

    // sidelengths of the square
    float uLength = calcVectorLength(u);
    float vLength = calcVectorLength(v);
    float uuLength = calcVectorLength(uu);
    float vvLength = calcVectorLength(vv);

    // cosine of the angles
    float uvCosTheta = calcCosAngle(uvDot, uLength, vLength);
    float uuvCosTheta = calcCosAngle(uuvDot, uuLength, vLength);
    float uvvCosTheta = calcCosAngle(uvvDot, uLength, vvLength);
    float uuvvCosTheta = calcCosAngle(uuvvDot, uuLength, vvLength);

    // test for approximately equal angles
    bool isSquareAngled = ((abs(uvCosTheta) < g_maxAngleDeviation) | (abs(uuvCosTheta) < g_maxAngleDeviation) |
                           (abs(uvvCosTheta) < g_maxAngleDeviation) | (abs(uuvvCosTheta) < g_maxAngleDeviation));
    if (! isSquareAngled) {
#ifdef DEBUG
        std::cerr << "marker dismissed: the angles are too far off 90 degrees" << std::endl;
#endif
        return false;
    }
    // test for approximately equal sidelengths
    bool hasEqualSidelength = ((abs(uLength - vLength)/uLength < g_maxLengthDeviation) |
                               (abs(uLength - uuLength)/uLength < g_maxLengthDeviation) |
                               (abs(uLength - vvLength)/uLength < g_maxLengthDeviation));
    if (! hasEqualSidelength) {
#ifdef DEBUG
        std::cerr << "marker dismissed: the marker does not have equal sidelengths)" << std::endl;
#endif
        return false;
    }

    // tests passed, return true
    return true;
}

//
//*          Extracts the robot id from the marker data
//*_____________________________________________________________________________________________________________________
//*
//*          resultData is stored in the order as seen in the figure below
//*          _________       _________       marker bits have the following properties:
//*                                          D = data
//*          | 0 1 2 |       | D 1 D |       0 = always white ('off')
//*          | 3 4 5 |       | P D P |       1 = always black ('on'), this is the direction of the marker
//*          | 6 7 8 |       | D 0 D |       P = parity:
//*          _________       _________       "n(D) even -> P = 1" | "n(D) odd -> P = 0"
//*
//*          rotation of the marker is unknown, so the direction bit can be in spot 1, 3, 5, 7, hereafter called 'case'
//*          the data markers are stored relative to this direction bit, in the following order:
//*          left_bottom -> left_top -> middle -> right_bottom -> right_top. the id can be calculated like this:
//*          id +=     1 +         2 +       4 +             8 +         16, if that certain bit is black ('on')
//*          below the order for each case:  ___________________________________________________________________________
//*          case 1: 6,0,4,8,2              | example:  | (id = 24, rotation = "case 3") -> resultData[3] = 1
//*          case 3: 8,6,4,2,0              |           | binary => 11000  -> resultData[2] = 'on', resultData[0] = 'on'
//*          case 5: 0,2,4,6,8              |           | n(D) even => P=1 -> resultData[7] = 'on', resultData[1] = 'on'
//*          case 7: 2,8,4,0,6              |           | } result:        -> [0,1,2,3,7] = 'on', [4,5,6,8] = 'off'
//*
//*/
bool ArucoMarkerfinder::getRobotID(std::vector<bool> &resultData, std::vector<int> &orientation, int &id) {

    int parity = 0;
    int startIndex = 0;
    int factor = 0;

    bool bitOne = resultData[1], bitThree = resultData[3], bitFive = resultData[5], bitSeven = resultData[7];

    // case 1: if bitOne = '1' and the opposing bit bitSeven = '0', and the two parity bits are the same
    if (bitOne && ! bitSeven && ((bitThree && bitFive) || (! bitThree && ! bitFive))) {
        orientation = {0, 3, 2, 1};         // orientation is used to calculate the angle later in getAngle()
        startIndex = 6;                     // we use startIndex and factor for the order of the cases. in case 1:
        factor = 4;                         // (6 + 4 * i)%10 = {6,10,14,18,22}%10 = {6,0,4,8,2}
        if (bitThree && bitFive) parity ++; // increase parity if the parity bits are both on

    }

        // case 3:
    else if (bitThree && ! bitFive && ((bitOne && bitSeven) || (! bitOne && ! bitSeven))) {
        orientation = {0, 2, 3, 1};
        startIndex = 8;
        factor = - 2;
        if (bitOne && bitSeven) parity ++;

    }

        // case 5:
    else if (bitFive && ! bitThree && ((bitOne && bitSeven) || (! bitOne && ! bitSeven))) {
        orientation = {1, 3, 2, 0};
        startIndex = 0;
        factor = 2;
        if (bitOne && bitSeven) parity ++;

    }

        // case 7:
    else if (bitSeven && ! bitOne && ((bitThree && bitFive) || (! bitThree && ! bitFive))) {
        orientation = {1, 2, 3, 0};
        startIndex = 2;
        factor = 6;
        if (bitThree && bitFive) parity ++;

    }
    else {
#ifdef DEBUG
        std::cerr << "marker dismissed: parity/direction combination incorrect" << std::endl;
#endif
        return false;
    }
    // for the 5 data bits, check their data and add their value to id if they are 'on'
    for (int i = 0; i < 5; i ++) {
        int D = (startIndex + i*factor)%10;
        if (resultData[D]) {
            id += (int) pow(2, i);
            parity ++;
        }
    }

    // check if the parity is odd, else the parity of the marker is wrong
    if (parity%2 == 0) {
#ifdef DEBUG
        std::cerr << "marker dismissed: parity incorrect" << std::endl;
#endif
        return false;
    }
    // all parity data is correct
    return true;
}

//
//*          Calculate the center pixel of the marker
//*_____________________________________________________________________________________________________________________
//*
//*    calculate the center of the square using vectors:
//*
//*     a ____ d        vector p = a + t(b-a);      vector q = c + s(d-c);
//*      |    |
//*      |____|     ->  center of the square is the intersection between the two vectors.
//*     c      b
//*                      s =    ((aY-cY)(bX-aX) + (cX-aX)(bY-aY) ) / ( (dY-cY)(bX-aX) + (cX-dX)(bY-aY))
//*                xCenter =    cX + s(dX - cX)
//*                yCenter =    cY + s(dY - cY)
//*/
bool ArucoMarkerfinder::getCenter(float &xCenter, float &yCenter, std::vector<int> corners) {

    auto aX = (float) corners[0], bX = (float) corners[2], cX = (float) corners[4], dX = (float) corners[6];
    auto aY = (float) corners[1], bY = (float) corners[3], cY = (float) corners[5], dY = (float) corners[7];

    if ( (dY - cY)*(bX - aX) + (cX - dX)*(bY - aY) == 0 ) {
#ifdef DEBUG
        std::cerr << "marker dismissed, divide by zero??: " << (dY - cY)*(bX - aX) + (cX - dX)*(bY - aY)
                  << std::endl;
#endif
        return false;
    }
    // calculate s and fill it in in the formulas for xCenter and Ycenter, explanation above
    float s = ( ((aY - cY)*(bX - aX) + (cX - aX)*(bY - aY)) /
                ((dY - cY)*(bX - aX) + (cX - dX)*(bY - aY)) );
    xCenter = cX + s*(dX - cX);
    yCenter = cY + s*(dY - cY);

    return true;
}

/// Gets the angle of the marker
void ArucoMarkerfinder::getAngle(float &angle, std::vector<int> &orientation, std::vector<int> &corners) {

//      the rotation of the marker is calculated by taking the two corners closest to the direction "1" bit
//      the direction is calculated by the average vector of the two corners closest to the "1" bit
//          and the corners furthest away from this bit.

    float xVector;
    float yVector;
    std::vector<float> thetaVector;

// using the two points closest to the "1" bit (orientation vector), calculate the direction
    for (int i = 0; i < 2; i ++) {
        xVector = corners[(2*orientation[2*i])] - corners[(2*orientation[2*i + 1])];
        yVector = corners[(2*orientation[2*i] + 1)] - corners[(2*orientation[2*i + 1] + 1)];

        if (xVector > 0) thetaVector.push_back(atan(yVector/xVector));
        else if (xVector < 0) thetaVector.push_back((float) (CV_PI + atan(yVector/xVector)));
        else thetaVector.push_back((float) (CV_PI*0.5));
    }
    angle = (float) (3.5*CV_PI + (thetaVector[0] + thetaVector[1])*0.5);
    while (angle > 2*CV_PI) angle -= 2*CV_PI;
    angle = - angle + CV_PI;
}

/// Checks if a pixel is white. if it is, find pixels around it and make a blob of white
void ArucoMarkerfinder::findWhiteBlob(int i, int j, std::vector<int> &x, std::vector<int> &y,
                                      std::vector<int> &index, Mat image) {

    auto &color = image.at<Vec3b>(i, j);
    Vec3b lowerBound = g_lowerWhiteMargin, upperBound = g_upperWhiteMargin;
    if (isColor(color, lowerBound, upperBound)) {
        // if a white pixels is found, create a grouped blob of pixels,
        Vec3b setColor = {0, 255, 0}; // set it Green so they are grouped
        groupWhitePixels(i, j, x, y, lowerBound, upperBound, image, setColor);
        index.push_back((int) x.size());
    }
}

/// Recursively groups white pixels
/// Breath first search
void ArucoMarkerfinder::groupWhitePixels(int i, int j, std::vector<int> &x, std::vector<int> &y,
                                         Vec3b lowerBound, Vec3b upperBound, Mat image, Vec3b setColor) {

    std::queue<int> xQueue, yQueue;
    xQueue.push(i);
    yQueue.push(j);

    while (! xQueue.empty()) {
        i = xQueue.front();
        j = yQueue.front();
        x.push_back(i);
        y.push_back(j);
        if ((i > g_pixFromEdge) && (j > g_pixFromEdge) && (i < image.rows-g_pixFromEdge) && (j < image.cols-g_pixFromEdge)) {

            int ii = 0;
            for (int jj : {1, 0, - 1, 0}) {          // check 'up left down right'
                Vec3b color = image.at<Vec3b>(i - ii, j - jj);
                if (isColor(color, lowerBound, upperBound)) {
                    image.at<Vec3b>(i - ii, j - jj) = setColor;

                    xQueue.push(i - ii);
                    yQueue.push(j - jj);
                }
                ii = jj;
            }
        }
        xQueue.pop();
        yQueue.pop();
    }

}

/// For each group of pixels, check if it actually is an aruco marker
bool ArucoMarkerfinder::checkIfSquareBlob(std::vector<int> &x, std::vector<int> &y, int startIndex, int endIndex,
                                          Mat image, std::vector<int> &corners,
                                          std::vector<int> &u, std::vector<int> &v, std::vector<int> &uu, std::vector<int> &vv) {

    if (endIndex - startIndex < g_minMarkerPixels) {
#ifdef DEBUG
        std::cerr << "marker dismissed: not enough pixels" << std::endl;
#endif
        return false;
    }
    Vec3b lowerBound = g_lowerWhiteMargin - g_deltaWhiteMargin;
    Vec3b upperBound = g_upperWhiteMargin;
    Vec3b setColor = {220, 220, 0};

    float xCenter;
    float yCenter;

    // find the pixel furthest away from the starting index and push the index back into corners
    findOppositeCorner(x[startIndex], y[startIndex], startIndex, endIndex, x, y, lowerBound, upperBound,
                       corners, image, setColor);

    // find the pixel furthest away from the pixel found above and push the index back into corners
    findOppositeCorner(corners[0], corners[1], startIndex, endIndex, x, y, lowerBound, upperBound,
                       corners, image, setColor);

    // find the center pixel of the first two corners
    float xabDst = corners[0] - corners[2];
    float yabDst = corners[1] - corners[3];
    xCenter = 0.5f*xabDst + corners[2];
    yCenter = 0.5f*yabDst + corners[3];

    // find the third corner
    findOppositeCorner((int) (xCenter - yabDst), (int) (yCenter + xabDst), startIndex, endIndex, x, y,
                       lowerBound, upperBound, corners, image, setColor);

    // find the last corner, which is the furthest pixel from the corner found above
    findOppositeCorner(corners[4], corners[5], startIndex, endIndex, x, y, lowerBound, upperBound,
                       corners, image, setColor);

    // save the corner data to corners
    for (int i = 0; i < 4; i ++) {
        // visualization of the corners in color blue
        image.at<Vec3b>(corners[2*i], corners[2*i + 1]) = {0, 0, 255};
    }

    // check if the marker is square
    createVectors(x, y, corners, u, v, uu, vv);
    bool isSquare = isSquareMarker(u, v, uu, vv);

    return isSquare;
}

/// Using the four vectors describing the corners of a square, extract the bit-data of the (potential) marker
bool ArucoMarkerfinder::findMarkerData(std::vector<bool> &markerData, std::vector<int> &u, std::vector<int> &v,
                                       std::vector<int> &uu, std::vector<int> &vv, std::vector<int> &corners, Mat image) {

    // extract marker bit data
    double xPixel;
    double yPixel;
    Vec3b green = {0, 255, 0}; //call Thijs at: 0629273740 for moral and emotional support
    int edgeCheck = 0;
    for (int i = - 1; i < ARUCOSIZE + 1; i ++) {
        for (int j = - 1; j < ARUCOSIZE + 1; j ++) {
            // calculate the position of all the marker data bits using a transformation from vectors to xy-plane of the image-pixels
            xPixel = corners[0] - ((j + 1.5)*((ARUCOSIZE - i + 0.5)*u[0] + (i + 1.5)*uu[0]) +
                                   (i + 1.5)*((ARUCOSIZE - j + 0.5)*v[0] + (j + 1.5)*vv[0]))/
                                  ((ARUCOSIZE + 2)*(ARUCOSIZE + 2));
            yPixel = corners[1] - ((j + 1.5)*((ARUCOSIZE - i + 0.5)*u[1] + (i + 1.5)*uu[1]) +
                                   (i + 1.5)*((ARUCOSIZE - j + 0.5)*v[1] + (j + 1.5)*vv[1]))/
                                  ((ARUCOSIZE + 2)*(ARUCOSIZE + 2));
            if ( xPixel < g_pixFromEdge || xPixel > image.rows-g_pixFromEdge ||
                 yPixel < g_pixFromEdge || yPixel > image.cols-g_pixFromEdge ) {
#ifdef DEBUG
                std::cerr << "marker dismissed: bitdata of the marker is outside the image frame" << std::endl;
#endif
                return false;
            }
            auto &color = image.at<Vec3b>((int) round(xPixel), (int) round(yPixel));
            if (i == - 1 || j == - 1 || i == ARUCOSIZE || j == ARUCOSIZE) {
                // at the edges, check if (most) pixels are white. if not, dismiss marker.
                if (color == green) {
                    image.at<Vec3b>((int) round(xPixel), (int) round(yPixel)) = {0, 225, 0};
                }
                else {
                    image.at<Vec3b>((int) round(xPixel), (int) round(yPixel)) = {0, 0, 0};
                    edgeCheck ++;
                    if (edgeCheck > 2) {
#ifdef DEBUG
                        std::cerr << "marker dismissed: the edge is not fully white" << std::endl;
#endif
                        return false;
                    }
                }
            }
            else if (color == green) {
                // in the center data structure, save the data of the bits
                image.at<Vec3b>((int) round(xPixel), (int) round(yPixel)) = {75, 0, 0};
                markerData.push_back(false);
            }
            else {
                image.at<Vec3b>((int) round(xPixel), (int) round(yPixel)) = {225, 0, 0};
                markerData.push_back(true);
            }
        }
    }

    return true;
}

/// Finds robot id, position and angle of the (potential) marker using the four corners of the marker
bool ArucoMarkerfinder::findRobotData(std::vector<bool> &resultData, std::vector<float> &posRotID,
                                      std::vector<int> &corners) {

    std::vector<int> orientation;

    int id = 0;
    bool validID = getRobotID(resultData, orientation, id);
    if (! validID) return false;

    posRotID.push_back(id);

    float xCenter, yCenter;
    bool centerExists = getCenter(xCenter, yCenter, corners);
    if (! centerExists) return false;

    posRotID.push_back(xCenter);
    posRotID.push_back(yCenter);

    float angle;
    getAngle(angle, orientation, corners);

    posRotID.push_back(angle);

    // all tests passed! yay, we got ourselves a robot!
    return true;
}

/// Finds the robot id, x,y position and rotations of all robots
void ArucoMarkerfinder::findMarkers(Mat image, std::vector<int> &markerID, std::vector<int> &markerX,
                                    std::vector<int> &markerY, std::vector<float> &markerTheta) {

    // create vector of all x- and y-positions of the white pixels
    std::vector<int> xSqMarker, ySqMarker;
    std::vector<int> startIndSqMarker = {0};

    // iterate through all pixels, skipping g_skipPixels- pixels, and find white ones.
    Vec3b green = {0, 255, 0};
    for (int i = g_pixFromEdge; i < image.rows - g_pixFromEdge; i += g_skipPixels) {
        for (int j = g_pixFromEdge; j < image.cols - g_pixFromEdge; j += g_skipPixels) {
            // if the pixel is not yet part of a square marker
            auto &color = image.at<Vec3b>(i, j);
            if (color != green) {
                findWhiteBlob(i, j, xSqMarker, ySqMarker, startIndSqMarker, image);

            }
        }
    }

    // for all groups of pixels found
    for (int marker = 0; marker < (int) startIndSqMarker.size() - 1; marker ++) {

        int startInd = startIndSqMarker[marker];
        int endInd = startIndSqMarker[marker + 1];

        bool isSquare, edgeIsWhite, isRobotID;
        std::vector<int> corners;
        std::vector<int> u, v, uu, vv;
        std::vector<bool> markerData;
        std::vector<float> posRotID;
        isSquare = checkIfSquareBlob(xSqMarker, ySqMarker, startInd, endInd, image, corners, u, v, uu, vv);

        if (! isSquare) continue;
        edgeIsWhite = findMarkerData(markerData, u, v, uu, vv, corners, image);
        if (! edgeIsWhite) continue;
        isRobotID = findRobotData(markerData, posRotID, corners);
        if (! isRobotID) continue;

        auto id = (int) posRotID[0];
        auto x = (int) round(posRotID[1]);
        auto y = (int) round(posRotID[2]);
        float theta = posRotID[3];

        markerID.push_back(id);
        markerX.push_back(y);
        markerY.push_back(x);
        markerTheta.push_back(theta);

        posRotID.clear();
        corners.clear();

    }

}

