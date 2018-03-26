//
// Created by wouter on 3/26/18.
//

#include <camera_calibration.h>
#include "plugin_detect_aruco.h"

plugin_detect_aruco::plugin_detect_aruco(FrameBuffer * _buffer, const CameraParameters& camera_params, const RoboCupField& field)
        : VisionPlugin(_buffer), camera_parameters(camera_params), field(field)
{
    _settings = new VarList("Robot Aruco Detection");

}

string plugin_detect_aruco::getName() {
    return "DetectArucoRobots";
}

VarList *plugin_detect_aruco::getSettings() {
    return _settings;
}

ProcessResult plugin_detect_aruco::process(FrameData *data, RenderOptions *options) {

}


