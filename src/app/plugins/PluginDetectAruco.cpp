//
// Created by wouter on 3/26/18.
//

#include <messages_robocup_ssl_detection.pb.h>
#include "PluginDetectAruco.h"

#define DEBUG

PluginDetectAruco::PluginDetectAruco(FrameBuffer * _buffer, const CameraParameters& camera_params, const RoboCupField& field)
        : VisionPlugin(_buffer), camera_parameters(camera_params), field(field)
{
    detector = new Detector(0,1);

    _settings = new VarList("Robot Aruco Detection");

}

string PluginDetectAruco::getName() {
    return "DetectArucoRobots";
}

VarList *PluginDetectAruco::getSettings() {
    return _settings;
}

ProcessResult PluginDetectAruco::process(FrameData *data, RenderOptions *options) {
    (void)options;
    if (data==0) return ProcessingFailed;

    SSL_DetectionFrame * detection_frame = 0;

    detection_frame=(SSL_DetectionFrame *)data->map.get("ssl_detection_frame");
    if (detection_frame == 0) detection_frame=(SSL_DetectionFrame *)data->map.insert("ssl_detection_frame",new SSL_DetectionFrame());




    vector<PosRotId> results = detector->performTrackingOnImage(cv::Mat(
            data->video.getHeight(),
            data->video.getWidth(),
            CV_8UC3,
            data->video.getData()), false);
    detection_frame->clear_robots_blue();
    detection_frame->clear_robots_yellow();

    auto robots_blue = detection_frame->mutable_robots_blue();
    auto robots_yellow = detection_frame->mutable_robots_yellow();
    for (PosRotId pri : results)
    {
        SSL_DetectionRobot*  robot = 0;
        if(pri.getID() >= 16) {
            robot = robots_blue->Add();
            robot->set_robot_id((unsigned int)(pri.getID()-16));
        } else {
            robot = robots_yellow->Add();
            robot->set_robot_id((unsigned int)pri.getID());
        }
        robot->set_x((float)pri.getX());
        robot->set_y((float)pri.getY());
        robot->set_confidence(1);
        robot->set_orientation((float)pri.getTheta());
        robot->set_height(0);
        robot->set_pixel_x((float)pri.getX());
        robot->set_pixel_y((float)pri.getY());
#ifdef DEBUG
        std::cout << "Detected robot " << robot->robot_id() << " at (" << robot->x() << ", " << robot->y() << ");" << endl;
#endif
    }

    return ProcessingOk;


}

PluginDetectAruco::~PluginDetectAruco() {

}

void PluginDetectAruco::postProcess(FrameData *data, RenderOptions *options) {
    VisionPlugin::postProcess(data, options);
}


