//
// Created by thijs on 27-6-19.
//

#ifndef SSL_VISION_PLUGINTECHINCALCHALLENGE_H
#define SSL_VISION_PLUGINTECHINCALCHALLENGE_H

#include "plugin_detect_balls.h"

class PluginDetectBallsSettings;
class PluginTechnicalChallenge : public PluginDetectBalls {

    protected:
        void image2relativeToRobot(GVector::vector3d<double> &rtr, const CMVision::Region * reg) const;
    public:
        PluginTechnicalChallenge(FrameBuffer * _buffer, LUT3D * lut, const CameraParameters& camera_params, const RoboCupField& field, PluginDetectBallsSettings * _settings=0);

        ~PluginTechnicalChallenge();


        virtual ProcessResult process(FrameData * data, RenderOptions * options);
        virtual VarList * getSettings();
        virtual string getName();
        bool isValidPosition(const CMVision::Region* reg);
};

#endif //SSL_VISION_PLUGINTECHINCALCHALLENGE_H
