//
// Created by Wouter Timmermans on 14-09-17.
//

#include "aruco_detector.h"

#define DEBUG

ArucoDetector::ArucoDetector(int total_markers, int bits) {
//    originMarker = originID;
//    limitMarker = limitID;

    setDictionaryProperties(total_markers, bits);




    totalTime = 0;
    totalIterations = 0;


    // Populate the camera optical parameters. These are unique per camera and should be recalibrated if the camera focus
    // changes
    cv::Ptr<cv::Mat> camPtr, distPtr;

    camPtr = cv::makePtr<cv::Mat>();
    distPtr = cv::makePtr<cv::Mat>();

    readCameraParameters(camPtr, distPtr);
//    grid_height = 1024;
//    grid_width = 1280;

    cameraMatrix = *camPtr;
    distCoeffs = *distPtr;
//    detectorParams->minMarkerPerimeterRate = 0.05;
//    detectorParams->maxMarkerPerimeterRate = 0.2;
//    detectorParams->adaptiveThreshWinSizeStep = 15;
//    detectorParams->adaptiveThreshWinSizeMin = 10;
//    detectorParams->adaptiveThreshWinSizeMax = 10;


}


std::vector<PosRotId> ArucoDetector::performTrackingOnImage(cv::Mat image, bool showDebug) {

    std::vector<PosRotId> result = std::vector<PosRotId>();

    if (!image.empty()) {
        dict_mutex.lock();
        auto tick = (double) cv::getTickCount();

        std::vector<int> markerIds;
        std::vector<int> markerX;
        std::vector<int> markerY;
        std::vector<float> markerTheta;

        finder.findMarkers(image, markerIds, markerX, markerY, markerTheta);


        dict_mutex.unlock();
        if (!markerIds.empty()) {
            if (showDebug) {
                printf("%-12s%-12s%-12s%-12s\n", "ID", "x (pixels)", "y (pixels)", "angle (pi rad)");

            }
            for (int i = 0; i < (int)markerIds.size(); i++) {
                int id = markerIds[i];
                int x = markerX[i];
                int y = markerY[i];
                float angle = markerTheta[i];

                PosRotId posRot = PosRotId(id, x, y, angle);
                result.insert(result.end(), posRot);

                std::string strID = std::to_string( id );
                std::string strX = std::to_string( x );
                std::string strY = std::to_string( y );
                std::string strT = std::to_string( (angle) );

                char const *idid = strID.c_str();
                char const *xx = strX.c_str();
                char const *yy = strY.c_str();
                char const *theta = strT.c_str();

                if (showDebug) {
                    printf("%-12s%-12s%-12s%-12s\n", idid, xx, yy, theta);
                }


            }
            if (showDebug) {
                std::cerr << std::endl << "========= END OF DETECTION ========" << std::endl << std::endl;
            }
        }
//        if (showDebug) {
//            cv::Mat imageCopy;
//            image.copyTo(imageCopy);
//
//                for (int i = 0; i < markerIds.size(); i++) {
//                    cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
//                    cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rotationVectors[i], translationVectors[i],
//                                        0.1);
//                }
//
//            cv::imshow("out", imageCopy);
//            //cv::waitKey(10);
//
//        }

        double currentTime = ((double) cv::getTickCount() - tick) / cv::getTickFrequency();
        totalTime += currentTime;
        totalIterations++;
        if (totalIterations % 30 == 0) {
            std::cout << "Detection Time = " << currentTime * 1000 << " ms "
                      << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << std::endl;
        }
    }
    return result;
}

cv::Point2f
ArucoDetector::calculatePosition(std::vector<cv::Point2f> observedPos) {

    // Calculate the center of the marker in camera coordinates
    // This is accomplished by calculating the point with x coordinate half between two opposing corners
    // And y coordinate half between two opposing corners.
    float deltax02 = observedPos[2].x - observedPos[0].x;
    float deltay02 = observedPos[2].y - observedPos[0].y;
    float centerX = observedPos[0].x + (deltax02 / 2);
    float centerY = observedPos[0].y + (deltay02 / 2);
    cv::Point2f markerCenter = cv::Point2f(centerX, centerY);
    return markerCenter;

    // We know the center of the marker in surface area coordinates. Now we calculate the center of the surfacebot from
    // this data. We know that the center of the bot lies on the circle with diameter MARKER_OFFSET with origin in the
    // center of the marker. To calculate the x and y position on this circle we use the cosinus and sinus functions
    // respectively;
//
//    double centerBotx = centerMarker.x + std::cos(angleZ + M_PI) * SURFBOTTRACKING_MARKER_OFFSET;
//    double centerBoty = centerMarker.y + std::sin(angleZ + M_PI) * SURFBOTTRACKING_MARKER_OFFSET;
//
//    return cv::Point2f((float) centerBotx, (float) centerBoty);
}


void ArucoDetector::readCameraParameters(cv::Ptr<cv::Mat> cameraMatrix, cv::Ptr<cv::Mat> distCoeffs) {
    //TODO: read these paramaters from a file
    double distArr[] = {-8.2549974582253244e-02, -1.5181783979810068e+00,
                        -4.9753678549509261e-04, 4.7107295904877293e-03,
                        4.0871908247047974e+00};
    cv::Matx33d camMatx(8.6298813107174658e+02, 0., 6.7620649647659229e+02, 0.,
                        8.6553882635457478e+02, 4.7027690275075901e+02, 0., 0., 1.);
    cv::Mat camMat = cv::Mat(cv::Size(3, 3), CV_64FC1);
    camMat = cv::Mat(camMatx * camMatx.t());
    cv::Mat distMat = cv::Mat(cv::Size(5, 1), CV_64FC1, distArr);
    *cameraMatrix = camMat;
    *distCoeffs = distMat;
}

double ArucoDetector::euclideanDist(cv::Point2f a, cv::Point2f b) {
    cv::Point2f diff = a - b;
    return cv::sqrt(diff.x * diff.x + diff.y * diff.y);
}

double ArucoDetector::getAvgRes(std::vector<cv::Point2f> corners) {
    double res = 0.0;
    if (!corners.empty() && corners.size() > 1) {
        double total = 0.0;
        for (int i = 0; i < corners.size() - 1; i++) {
            total += euclideanDist(corners[i], corners[i + 1]);
        }
        total += euclideanDist(corners[corners.size() - 1], corners[0]);
        res = total / corners.size();
    }
    return res;
}

void ArucoDetector::setDictionaryProperties(int total_markers, int bits) {
    dict_mutex.lock();
    //dictionary = cv::aruco::Dictionary::create(total_markers, bits);
    //std::cout << "dictionary updated, is now " << dictionary->markerSize << std::endl;
    //usleep(10000);
    dict_mutex.unlock();
}
