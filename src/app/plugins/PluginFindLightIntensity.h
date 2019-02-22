//
// Created by thijs on 20-2-19.
//

#ifndef SSL_VISION_PLUGINFINDLIGHTINTENSITY_H
#define SSL_VISION_PLUGINFINDLIGHTINTENSITY_H

#include <visionplugin.h>
#include <iostream>
#include <string>
#include "opencv2/core/core.hpp"
#include <messages_robocup_ssl_detection.pb.h>
#include <VarNotifier.h>
#include <camera_calibration.h>

class PluginFindLightIntensitySettings {
        friend class PluginFindLightIntensity;

    protected:

        VarList * _settings;
        VarBool * is_enabled;
        VarInt * left_lower_x;
        VarInt * left_upper_x;
        VarInt * left_lower_y;
        VarInt * left_upper_y;
        VarInt * right_lower_x;
        VarInt * right_upper_x;
        VarInt * right_lower_y;
        VarInt * right_upper_y;

    public:
        PluginFindLightIntensitySettings() {
            _settings = new VarList("Light Intensity Settings");
            _settings->addChild(is_enabled = new VarBool("Enabled", true));
            _settings->addChild(left_lower_x = new VarInt("Left lower x-value", 50, 0));
            _settings->addChild(left_upper_x = new VarInt("Left upper x-value", 270, 0));
            _settings->addChild(left_lower_y = new VarInt("Left lower y-value", 50, 0));
            _settings->addChild(left_upper_y = new VarInt("Left upper y-value", 430, 0));
            _settings->addChild(right_lower_x = new VarInt("Right lower x-value", 370, 0));
            _settings->addChild(right_upper_x = new VarInt("Right upper x-value", 590, 0));
            _settings->addChild(right_lower_y = new VarInt("Right lower y-value", 50, 0));
            _settings->addChild(right_upper_y = new VarInt("Right upper y-value", 430, 0));
        }

        VarList * getSettings() {
            return _settings;
        }
};

class PluginFindLightIntensity : public VisionPlugin {
    private:
        unsigned char getTotalBrightnessInArea(FrameData* data, int lowerX, int upperX, int lowerY, int upperY);
    protected:
        PluginFindLightIntensitySettings * _settings;
        VarNotifier _notifier;
        short print;
        bool _is_enabled;
        unsigned int _left_lower_x;
        unsigned int _left_upper_x;
        unsigned int _left_lower_y;
        unsigned int _left_upper_y;
        unsigned int _right_lower_x;
        unsigned int _right_upper_x;
        unsigned int _right_lower_y;
        unsigned int _right_upper_y;

        const CameraParameters& camera_parameters;

    public:
        virtual ProcessResult process(FrameData * data, RenderOptions * options);
        virtual void postProcess(FrameData* data, RenderOptions* options);
        virtual std::string getName();
        virtual VarList * getSettings();

        PluginFindLightIntensity(FrameBuffer* _buffer, const CameraParameters &camera_params);

        ~PluginFindLightIntensity() {

        }
};

#endif //SSL_VISION_PLUGINFINDLIGHTINTENSITY_H
