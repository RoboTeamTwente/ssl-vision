
//
// Created by thijs on 2-10-18.
//

#include "aruco_markerfinder.h"

ArucoMarkerfinder::ArucoMarkerfinder() = default;

/// checks if a color is within certain bounds, input: (color, lower bound, upper bound)
bool ArucoMarkerfinder::isColor(Vec3b color, Vec3b lowerBound, Vec3b upperBound) {
    // assume a pixel is white, and disprove it.
    bool isColor = true;
    for (int c = 0; c < 3; c++) {
        if (color[c] < lowerBound[c] || color[c] > upperBound[c]) {
            isColor = false;
        }
    }
    return isColor;
}

/// finds the furthets pixel from vector 'relativeToPixel', looking in the selected indexes in vectors x & y
int ArucoMarkerfinder::findFurthestPixel(int xRelative, int yRelative, int startIndex, int endIndex,
                                         std::vector<int> &x, std::vector<int> &y) {
    int xDst, yDst, furthestIndex = startIndex;
    int distanceTest, distance = 0;
    for (int I = startIndex; I < endIndex; I++) {
        xDst = x[I] - xRelative;
        yDst = y[I] - yRelative;
        distanceTest = (xDst * xDst + yDst * yDst);
        if (distanceTest > distance) {
            distance = distanceTest;
            furthestIndex = I;
        }
    }
    return furthestIndex;
}

/// finds the opposite corner of a (presumably) square blob of pixels
std::vector<int> ArucoMarkerfinder::findOppositeCorner(int xRelative, int yRelative, int startIndex, int endIndex,
                                                       std::vector<int> &x, std::vector<int> &y,
                                                       Vec3b &lowerBound, Vec3b &upperBound, Mat image, Vec3b &setColor) {

    // find opposite corner within blob
    int furthestIndex = findFurthestPixel(xRelative, yRelative, startIndex, endIndex, x, y);

    // find a few new pixels with a wider color-margin in the corner
    int maxPixels = 30;
    std::vector<int> xCandidatePixels = {x[furthestIndex]}, yCandidatePixels = {y[furthestIndex]};
    groupWhitePixels(x[furthestIndex], y[furthestIndex], xCandidatePixels, yCandidatePixels, lowerBound, upperBound,
                     std::move(image), setColor, MAXRECURSION - maxPixels);

    // find the furthest corner again using these new pixels
    int candidateMaxIndex = (int) (xCandidatePixels.size() - 1);
    furthestIndex = findFurthestPixel(xRelative, yRelative, 0, candidateMaxIndex, xCandidatePixels, yCandidatePixels);
    std::vector<int> furthestPixel = {xCandidatePixels[furthestIndex], yCandidatePixels[furthestIndex]};
    return furthestPixel;
}

/// creates four vectors of the sides of a square
void ArucoMarkerfinder::createVectors(std::vector<int> &x, std::vector<int> &y, std::vector<int> &index,
                                      std::vector<int> &v0, std::vector<int> &v1, std::vector<int> &v2,
                                      std::vector<int> &v3) {

    v0 = {index[0] - index[4], index[1] - index[5]};
    v1 = {index[0] - index[6], index[1] - index[7]};
    v2 = {index[6] - index[2], index[7] - index[3]};
    v3 = {index[4] - index[2], index[5] - index[3]};
}

/// calculate the dot product between two vectors
int ArucoMarkerfinder::calcDotProduct(std::vector<int> &v1, std::vector<int> &v2) {
    int dot = v1[0] * v2[0] + v1[1] * v2[1];
    return dot;
}

/// calculate the length of a vector
float ArucoMarkerfinder::calcVectorLength(std::vector<int> &v1) {
    float lengthSquared = v1[0] * v1[0] + v1[1] * v1[1];
    float length = sqrt(lengthSquared);
    return length;
}

/// calculate the cosine of the angle between two vectors
float ArucoMarkerfinder::calcCosAngle(float dot, float v0Length, float v1Length) {
    float cosAngle = dot / (v0Length * v1Length);
    return cosAngle;
}

bool ArucoMarkerfinder::getRobotID(std::vector<bool> &resultData, std::vector<int> &orientation, int &id) {

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

    int parity = 0, startI = 0, factor = 0;

    bool one = resultData[1], three = resultData[3], five = resultData[5], seven = resultData[7];
    // case 1:
    if ( one && !seven && ( (three && five) || (!three && !five) ) ) {
        orientation = {0, 3, 2, 1};
        startI = 6;
        factor = 4;
        if (three && five) parity++;

    } else if ( three && !five && ( (one && seven) || (!one && !seven) ) ) {
        orientation = {0, 2, 3, 1};
        startI = 8;
        factor = -2;
        if (one && seven) parity++;

    } else if ( five && !three && ( (one && seven) || (!one && !seven) ) ) {
         orientation = {1, 3, 2, 0};
        startI = 0;
        factor = 2;
        if (one && seven) parity++;

    } else if ( seven && !one && ( (three && five) || (!three && !five) ) ) {
        orientation = {1, 2, 3, 0};
        startI = 2;
        factor = 6;
        if (three && five) parity++;

    } else {
        std::cerr << "marker dismissed: parity/direction combination incorrect" << std::endl;
        return false;
    }

    for (int i = 0; i < 5; i++) {
        int D = (startI + i * factor) % 10;
        if (resultData[D]) {
            id += (int) pow(2, i);
            parity++;
        }
    }

    if (parity % 2 == 0) {
        std::cerr << "marker dismissed: parity incorrect" << std::endl;
        return false;
    }
    // all parity data is correct
    return true;
}

bool ArucoMarkerfinder::getCenter(float &xCenter, float &yCenter, std::vector<int> corners) {

    auto ax = (float) corners[0], bx = (float) corners[2], cx = (float) corners[4], dx = (float) corners[6];
    auto ay = (float) corners[1], by = (float) corners[3], cy = (float) corners[5], dy = (float) corners[7];

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
        xCenter = cx + s * (dx - cx);
        yCenter = cy + s * (dy - cy);

    } else {
        std::cerr << "marker dismissed, divide by zero??: " << (dy - cy) * (bx - ax) + (cx - dx) * (by - ay)
                  << std::endl;
        return false;
    }
    // normally this should return true..
    return true;
}

void ArucoMarkerfinder::getAngle(float &angle, std::vector<int> &orientation, std::vector<int> &corners) {

//      the rotation of the marker is calculated by taking the two corners closest to the direction "1" bit
//      the direction is calculated by the average average vector of the two corners closest to the "1" bit
//          and the corners furthest away from this bit.

    float xVector;
    float yVector;
    std::vector<float> thetaVector;

// using the two points closest to the "1" bit (orientation vector), calculate the direction
    for (int i = 0; i < 2; i++) {
        xVector = corners[(2 * orientation[2 * i])] - corners[(2 * orientation[2 * i + 1])];
        yVector = corners[(2 * orientation[2 * i] + 1)] - corners[(2 * orientation[2 * i + 1] + 1)];

        if (xVector > 0) thetaVector.push_back(atan(yVector / xVector));
        else if (xVector < 0) thetaVector.push_back((float) (CV_PI + atan(yVector / xVector)));
        else thetaVector.push_back((float) (CV_PI * 0.5));
    }
    angle = (float) (2 * CV_PI + (thetaVector[0] + thetaVector[1]) * 0.5);
    while (angle > CV_PI) angle -= 2 * CV_PI;
}

/// checks if a pixel is white. if it is, find pixels around it and make a blob of white
void ArucoMarkerfinder::findWhiteBlob(int i, int j, std::vector<int> &x, std::vector<int> &y,
                                      std::vector<int> &index, Mat image) {

    auto &color = image.at<Vec3b>(i, j);
    Vec3b lowerBound = g_LowerWhiteMargin, upperBound = g_upperWhiteMargin;
    if (isColor(color, lowerBound, upperBound)) {
        // if a white pixels is found, create a grouped blob of pixels,
        Vec3b setColor = {0, 255, 0};
        groupWhitePixels(i, j, x, y, lowerBound, upperBound, image, setColor, 0);
        index.push_back((int) x.size());
    }
}

/// recursively groups white pixels
void ArucoMarkerfinder::groupWhitePixels(int i, int j, std::vector<int> &x, std::vector<int> &y,
                                         Vec3b lowerBound, Vec3b upperBound, Mat image, Vec3b setColor, int iteration) {
    // push the current pixel data for this blob/square into a vector, so it can be used later
    x.push_back(i);
    y.push_back(j);

    // if whe are not at the edge.. check left,up,right,down for pixels in the current square
    if (iteration < MAXRECURSION && (i > 1) && (j > 1) && (i < image.rows - 1) && (j < image.cols - 1)) {
        // if the pixel at i-ii,j-jj is white, mark it with color green and check the squares around that pixel
        int ii = 0;
        for (int jj : {1, 0, -1, 0}) {          // check 'up left down right'
            auto &MinColor = image.at<Vec3b>(i - ii, j - jj);
            if (isColor(MinColor, lowerBound, upperBound)) {
                //set color of the pixel to green and look for other white pixels around that pixel
                image.at<Vec3b>(i - ii, j - jj) = setColor;
                groupWhitePixels(i - ii, j - jj, x, y, lowerBound, upperBound, image, setColor, ++iteration);
            }
            ii = jj;
        }
    }
}

/// for each group of pixels, check if it actually is an aruco marker
bool ArucoMarkerfinder::checkIfSquareBlob(std::vector<int> &x, std::vector<int> &y, int startIndex, int endIndex,
                                          Mat image, std::vector<int> &corners,
                                          std::vector<int> &u, std::vector<int> &v, std::vector<int> &uu, std::vector<int> &vv) {

    if (endIndex - startIndex < g_minMarkerPixels) {
        std::cerr << "marker dismissed: not enough pixels" << std::endl;
        return false;
    }
    Vec3b lowerBound = g_LowerWhiteMargin - g_deltaWhiteMargin, upperBound = g_upperWhiteMargin;
    Vec3b setColor = {220, 220, 0};

    float xCenter, yCenter;
    std::vector<int> relativeToPixel = {0, 0};
    std::vector<int> furthestpixel;

    // find the pixel furthest away from the starting index and push the index back into corners

    furthestpixel = findOppositeCorner(x[startIndex], y[startIndex], startIndex, endIndex, x, y, lowerBound, upperBound,
                                       image, setColor);

    corners.push_back(furthestpixel[0]);
    corners.push_back(furthestpixel[1]);

    // find the pixel furthest away from the pixel found above and push the index back into corners
    furthestpixel = findOppositeCorner(corners[0], corners[1], startIndex, endIndex, x, y, lowerBound, upperBound,
                                       image, setColor);

    corners.push_back(furthestpixel[0]);
    corners.push_back(furthestpixel[1]);

    // find the center pixel of the first two corners
    float xabDst = corners[0] - corners[2];
    float yabDst = corners[1] - corners[3];
    xCenter = 0.5f * xabDst + corners[2];
    yCenter = 0.5f * yabDst + corners[3];

    // find the third corner
    furthestpixel = findOppositeCorner((int)(xCenter - yabDst), (int)(yCenter + xabDst), startIndex, endIndex, x, y,
                                       lowerBound, upperBound, image, setColor);

    corners.push_back(furthestpixel[0]);
    corners.push_back(furthestpixel[1]);

    // find the last corner, which is the furthest pixel from the corner found above
    furthestpixel = findOppositeCorner(corners[4], corners[5], startIndex, endIndex, x, y, lowerBound, upperBound,
                                       image, setColor);

    corners.push_back(furthestpixel[0]);
    corners.push_back(furthestpixel[1]);

    // save the corner data to corners
    for (int i = 0; i < 4; i++) {
        // visualization of the corners in color blue
        image.at<Vec3b>(corners[2*i], corners[2*i+1]) = {0, 0, 255};
    }

    // vector calculations to find the positions of the aruco bit-data

    // create vectors of the square sides
    createVectors(x, y, corners, u, v, uu, vv);

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
    bool isSquareAngled = ((abs(uvCosTheta) < g_maxAngleDeviation)  | (abs(uuvCosTheta) < g_maxAngleDeviation)  |
                           (abs(uvvCosTheta) < g_maxAngleDeviation) | (abs(uuvvCosTheta) < g_maxAngleDeviation) );
    if (!isSquareAngled) {
        std::cerr << "marker dismissed: the angles are too far off 90 degrees" << std::endl;
        return false;
    }
    // test for approximately equal sidelengths
    bool hasEqualSidelength = ((abs(uLength - vLength ) / uLength < g_maxLengthDeviation) |
                               (abs(uLength - uuLength) / uLength < g_maxLengthDeviation) |
                               (abs(uLength - vvLength) / uLength < g_maxLengthDeviation) );
    if (!hasEqualSidelength) {
        std::cerr << "marker dismissed: the marker does not have equal sidelengths)" << std::endl;
        return false;
    }

    // all checks passed, we are dealing with a square
    return true;
}

bool ArucoMarkerfinder::findMarkerData(std::vector<bool> &markerData, std::vector<int> &u, std::vector<int> &v,
                                       std::vector<int> &uu, std::vector<int> &vv, std::vector<int> &corners, Mat image) {

    // extract marker bit data
    double xx;
    double yy;
    Vec3b green = {0, 255, 0};
    int edgeCheck = 0;
    for (int i = -1; i < ARUCOSIZE + 1; i++) {
        for (int j = -1; j < ARUCOSIZE + 1; j++) {
            // calculate the position of all the marker data bits using a transformation from vectors to xy-plane of the image-pixels
            xx = corners[0] - ((j + 1.5) * ((ARUCOSIZE - i + 0.5) * u[0] + (i + 1.5) * uu[0]) +
                               (i + 1.5) * ((ARUCOSIZE - j + 0.5) * v[0] + (j + 1.5) * vv[0])) /
                              ((ARUCOSIZE + 2) * (ARUCOSIZE + 2));
            yy = corners[1] - ((j + 1.5) * ((ARUCOSIZE - i + 0.5) * u[1] + (i + 1.5) * uu[1]) +
                               (i + 1.5) * ((ARUCOSIZE - j + 0.5) * v[1] + (j + 1.5) * vv[1])) /
                              ((ARUCOSIZE + 2) * (ARUCOSIZE + 2));

            auto &color = image.at<Vec3b>((int) round(xx), (int) round(yy));
            if (i == -1 || j == -1 || i == ARUCOSIZE || j == ARUCOSIZE) {
                // at the edges, check if (most) pixels are white. if not, dismiss marker.
                if (color == green) {
                    image.at<Vec3b>((int) round(xx), (int) round(yy)) = {0, 225, 0};
                } else {
                    image.at<Vec3b>((int) round(xx), (int) round(yy)) = {0, 0, 0};
                    edgeCheck++;
                    if (edgeCheck > 2) {
                        std::cerr << "marker dismissed: the edge is not fully white" << std::endl;
                        return false;
                    }
                }
            } else if (color == green) {
                // in the center data structure, save the data of the bits
                image.at<Vec3b>((int) round(xx), (int) round(yy)) = {75, 0, 0};
                markerData.push_back(false);
            } else {
                image.at<Vec3b>((int) round(xx), (int) round(yy)) = {225, 0, 0};
                markerData.push_back(true);
            }
        }
    }

    return true;
}

/// finds robot id assigned to the aruco marker data
bool ArucoMarkerfinder::findRobotData(std::vector<bool> &resultData, std::vector<float> &posRotID,
                                      std::vector<int> &corners) {

    std::vector<int> orientation;
    int id = 0;
    bool validID = getRobotID(resultData, orientation, id);
    if (!validID) return false;
    posRotID.push_back(id);

    float xCenter, yCenter;
    bool centerExists = getCenter(xCenter, yCenter, corners);
    if (!centerExists) return false;

    posRotID.push_back(xCenter);
    posRotID.push_back(yCenter);

    float angle;
    getAngle(angle, orientation, corners);
    posRotID.push_back(angle);

    // all tests passed! yay, we got ourselves a robot!
    return true;

}

/// finds the robot id, x,y position and rotations of all robots
void ArucoMarkerfinder::findMarkers(Mat image, std::vector<int> &markerID, std::vector<int> &markerX,
                                    std::vector<int> &markerY, std::vector<float> &markerTheta) {

    // create vector of all x- and y-positions of the white pixels
    std::vector<int> xSqMarker, ySqMarker;
    std::vector<int> startIndSqMarker = {0};

    // iterate through all pixels, skipping g_skipPixels- pixels, and find white ones.
    Vec3b green = {0, 255, 0};
    for (int i = g_skipPixels + 1; i < image.rows - g_skipPixels; i += g_skipPixels) {
        for (int j = g_skipPixels + 1; j < image.cols - g_skipPixels; j += g_skipPixels) {
            // if the pixel is not yet part of a square marker
            auto &color = image.at<Vec3b>(i, j);
            if (color != green) {
                findWhiteBlob(i, j, xSqMarker, ySqMarker, startIndSqMarker, image);

            }
        }
    }

    // create vector for the data to send back

    // for all square markers found
    for (int marker = 0; marker < (int) startIndSqMarker.size() - 1; marker++) {
        // check if the thing we are dealing with is a square and extract the marker bit data
        int startInd = startIndSqMarker[marker];
        int endInd = startIndSqMarker[marker + 1];

        bool isSquare, edgeIsWhite, isRobotID;
        std::vector<int> corners;
        std::vector<int> u, v, uu, vv;
        std::vector<bool> markerData;
        std::vector<float> posRotID;
        isSquare = checkIfSquareBlob(xSqMarker, ySqMarker, startInd, endInd, image, corners, u, v, uu, vv);

        // if we are dealing with a square, extract the data from the marker and connect the marker bit data with the robot id
        if (!isSquare) continue;
        edgeIsWhite = findMarkerData(markerData, u, v, uu, vv, corners, image);
        if (!edgeIsWhite) continue;
        isRobotID = findRobotData(markerData, posRotID, corners);
        if (!isRobotID) continue;

        auto id = (int)posRotID[0];
        auto x = (int) round(posRotID[1]);
        auto y = (int) round(posRotID[2]);
        float theta = posRotID[3];

        markerID.push_back(id);
        markerX.push_back(x);
        markerY.push_back(y);
        markerTheta.push_back(theta);

        posRotID.clear();
        corners.clear();
    }
}

