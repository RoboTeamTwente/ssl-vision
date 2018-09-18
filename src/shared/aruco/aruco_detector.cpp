//
// Created by Wouter Timmermans on 14-09-17.
//

#include "aruco_detector.h"


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
    detectorParams->minMarkerPerimeterRate = 0.05;
    detectorParams->maxMarkerPerimeterRate = 0.2;
    detectorParams->adaptiveThreshWinSizeStep = 15;
    detectorParams->adaptiveThreshWinSizeMin = 10;
    detectorParams->adaptiveThreshWinSizeMax = 10;


}


std::vector<PosRotId> ArucoDetector::performTrackingOnImage(cv::Mat image, bool showDebug) {
    bool have_detection = false;

    std::vector<PosRotId> result = std::vector<PosRotId>();

    if (!image.empty()) {
        dict_mutex.lock();
        auto tick = (double) cv::getTickCount();

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f> > markerCorners, rejectedCandidates;
        std::vector<cv::Vec3d> rotationVectors, translationVectors;
        cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);
//        for (std::vector<cv::Point2f>& cornervec : markerCorners) {
//            std::reverse(cornervec.begin(), cornervec.end());
//        }

        for (int i = 0; i < markerIds.size(); i++) {
            rotationVectors.emplace_back(cv::Vec3d(1.,1.,1.));
        }

        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.085, cameraMatrix, distCoeffs, rotationVectors,
                                             translationVectors);

        dict_mutex.unlock();
        if (!markerIds.empty()) {
            // Convert the rodrigues parameter vectors from the estimatePoseSingleMarkers method to rotation matrices.
            // rotMatrices contains the rotation matrices for each observed marker.
            // These rotation matrices are easier to compare than the rodrigues parameters.
            std::vector<cv::Mat> rotMatrices = std::vector<cv::Mat>();

            for (int i = 0; i < markerIds.size(); i++) {
            //for (const auto &v : rotationVectors) {
                cv::Mat vMat;
                auto v = rotationVectors[i];
                cv::Rodrigues(v, vMat);
                rotMatrices.insert(rotMatrices.end(), vMat);
            }


            for (int i = 0; i < markerIds.size(); i++) {
                int id = markerIds[i];

                // Calculate the angle of the marker relative to the origin marker.
                // The delta matrix is calculated as follows:
                // ~[origin] * [marker]
                // Because the camera is perpendicular to the surface we are concerned with the rotation in the
                // Z axis. The rotation angle in the z axis can be calculated as the
                // atan2 of delta[21] and delta[11]

                cv::Mat originInv = rotMatrices[i];
                double elem21 = originInv.at<double>(1, 0);
                double elem11 = originInv.at<double>(0, 0);
                double angleZ = std::atan2(elem21, elem11);
                cv::Point2f pos = calculatePosition(markerCorners[i]);
                PosRotId posRot = PosRotId(id, pos.x, pos.y, angleZ);
                result.insert(result.end(), posRot);


                if (showDebug) {
                    std::cout << "Marker " << id << " at " << pos
                              << " with rotation " << angleZ << std::endl;
                }


            }
            if (showDebug) {
                std::cout << std::endl << "========= END OF DETECTION ========" << std::endl << std::endl;
            }
            have_detection = true;
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
    cv::Point2f centerMarker;

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
