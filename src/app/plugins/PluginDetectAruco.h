//
// Created by wouter on 3/26/18.
//

#ifndef SSL_VISION_PLUGIN_DETECT_ARUCO_H
#define SSL_VISION_PLUGIN_DETECT_ARUCO_H


#include <VarNotifier.h>
#include <detector.h>
#include "visionplugin.h"
#include <camera_calibration.h>

class PluginDetectAruco
: public VisionPlugin {
protected:
    VarNotifier _notifier;
    VarList * _settings;

    Detector * detector;

    const CameraParameters& camera_parameters;
    const RoboCupField& field;


public:
    ProcessResult process(FrameData * data, RenderOptions * options) override;
    void postProcess(FrameData * data, RenderOptions * options) override;
    VarList * getSettings() override;
    string getName() override;

    PluginDetectAruco(FrameBuffer *_buffer, const CameraParameters &camera_params, const RoboCupField &field);

    ~PluginDetectAruco();
};


#endif //SSL_VISION_PLUGIN_DETECT_ARUCO_H
