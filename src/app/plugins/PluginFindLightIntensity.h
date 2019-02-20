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

class PluginFindLightIntensity : public VisionPlugin {
    private:
        unsigned long long getTotalBrightnessInArea(FrameData* data, int lowerX, int upperX, int lowerY, int upperY);

    public:
        virtual ProcessResult process(FrameData * data, RenderOptions * options);
        virtual void postProcess(FrameData* data, RenderOptions* options);
        virtual std::string getName();

        PluginFindLightIntensity(FrameBuffer* _buffer)
                :VisionPlugin(_buffer) { };

        ~PluginFindLightIntensity() {

        }
};

#endif //SSL_VISION_PLUGINFINDLIGHTINTENSITY_H
