//
// Created by wouter on 3/26/18.
//


#include "plugin_detect_aruco.h"

#define DEBUG
plugin_detect_aruco::plugin_detect_aruco(FrameBuffer * _buffer, const CameraParameters& camera_params, const RoboCupField& field)
        : VisionPlugin(_buffer), camera_parameters(camera_params), field(field)
{


    _settings = new plugin_detect_aruco_settings();
    _notifier.addRecursive(_settings->getSettings());

    _enabled = _settings->isEnabled->getBool();
    _marker_bits = _settings->marker_bits->getInt();
    _total_markers = _settings->total_markers->getInt();
    _markers_per_team = _settings->markers_per_team->getInt();
    detector = new ArucoDetector(_total_markers, _marker_bits);

}

string plugin_detect_aruco::getName() {
    return "DetectArucoRobots";
}

VarList *plugin_detect_aruco::getSettings() {
    return _settings->getSettings();
}

ProcessResult plugin_detect_aruco::process(FrameData *data, RenderOptions *options) {
    (void)options;
    if (data==0) return ProcessingFailed;

    if (_notifier.hasChanged()) {
        _enabled = _settings->isEnabled->getBool();
        _marker_bits = _settings->marker_bits->getInt();
        _total_markers = _settings->total_markers->getInt();
        _markers_per_team = _settings->markers_per_team->getInt();
        detector->setDictionaryProperties(_total_markers, _marker_bits);

    }
    if (!_enabled) {
        return ProcessingOk;
    }

    SSL_DetectionFrame * detection_frame = 0;

    detection_frame=(SSL_DetectionFrame *)data->map.get("ssl_detection_frame");
    if (detection_frame == 0) detection_frame=(SSL_DetectionFrame *)data->map.insert("ssl_detection_frame",new SSL_DetectionFrame());




    cv::Mat img = cv::Mat(
            data->video.getHeight(),
            data->video.getWidth(),
            CV_8UC3,
            data->video.getData());

    //cv::cvtColor(img,img,cv::COLOR_RGB2GRAY);
    //cv::fastNlMeansDenoising(img, img);
    //cv::threshold(img,img,127,255,cv::THRESH_BINARY);

    //data->map.insert("aruco_frame",new cv::Mat(img));
    vector<PosRotId> results = detector->performTrackingOnImage(img, true);
    detection_frame->clear_robots_blue();
    detection_frame->clear_robots_yellow();

    auto robots_blue = detection_frame->mutable_robots_blue();
    auto robots_yellow = detection_frame->mutable_robots_yellow();
    for (PosRotId pri : results)
    {
        vector2d reg_img_center(pri.getX(),pri.getY());
        vector3d reg_center3d;
        camera_parameters.image2field(reg_center3d,reg_img_center,140.0);
        vector2d reg_center(reg_center3d.x,reg_center3d.y);
        SSL_DetectionRobot*  robot = 0;
        if(pri.getID() >= 16) {
            robot = robots_blue->Add();
            robot->set_robot_id((unsigned int)(pri.getID()-16));
        } else {
            robot = robots_yellow->Add();
            robot->set_robot_id((unsigned int)pri.getID());
        }
        robot->set_x((float)reg_center.x);
        robot->set_y((float)reg_center.y);
        robot->set_confidence(1);
        robot->set_orientation((float)-(pri.getTheta() - .5*CV_PI));
        if (robot->orientation() > (float)CV_PI) robot->set_orientation((robot->orientation()-(2*CV_PI)));
        robot->set_height(0);
        robot->set_pixel_x((float)pri.getX());
        robot->set_pixel_y((float)pri.getY());
#ifdef DEBUG
        std::cerr << "Detected robot " << robot->robot_id() << " at (" << robot->x() << ", " << robot->y() << ", " << robot->orientation() << ");" << endl;
#endif
    }
#ifdef DEBUG
    std::cerr << endl << "-----------------------" << endl;
#endif
    return ProcessingOk;


}

plugin_detect_aruco::~plugin_detect_aruco() {

}

void plugin_detect_aruco::postProcess(FrameData *data, RenderOptions *options) {
    VisionPlugin::postProcess(data, options);
}


