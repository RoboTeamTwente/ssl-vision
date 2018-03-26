//
// Created by wouter on 3/26/18.
//

#ifndef SSL_VISION_PLUGIN_DETECT_ARUCO_H
#define SSL_VISION_PLUGIN_DETECT_ARUCO_H


#include <VarNotifier.h>
#include "visionplugin.h"

class plugin_detect_aruco
: public VisionPlugin {
protected:
    VarNotifier _notifier;
    VarList * _settings;

    const CameraParameters& camera_parameters;
    const RoboCupField& field;


public:
    virtual ProcessResult process(FrameData * data, RenderOptions * options);
    virtual VarList * getSettings();
    virtual string getName();

    plugin_detect_aruco(FrameBuffer *_buffer, const CameraParameters &camera_params, const RoboCupField &field);
    ~plugin_detect_aruco();
};


#endif //SSL_VISION_PLUGIN_DETECT_ARUCO_H
