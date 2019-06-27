//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
  \file    plugin_detect_balls.h
  \brief   C++ Interface: plugin_detect_balls
  \author  Author Name, 2009
*/
//========================================================================
#ifndef PLUGIN_DETECT_BALLS_H
#define PLUGIN_DETECT_BALLS_H

#include <visionplugin.h>
#include "cmvision_region.h"
#include "messages_robocup_ssl_detection.pb.h"
#include "camera_calibration.h"
#include "field_filter.h"
#include "cmvision_histogram.h"
#include "vis_util.h"
#include "VarNotifier.h"
#include "lut3d.h"
#include <list>

/**
	@author Author Name
*/

class PluginDetectBallsSettings;

//Data structure for storing and sorting the filtered regions
class BallDetectResult {
    public:
        const CMVision::Region* reg;
        float conf;

        BallDetectResult(const CMVision::Region* reg, float conf) {
            this->reg = reg;
            this->conf = conf;
        }

        bool operator<(BallDetectResult a) {

            int areaDiff = a.reg->area - reg->area;
            int squarenessDiff = static_cast<int>((a.reg->squareness() - reg->squareness() )*10);

            int t = areaDiff + squarenessDiff;
            return t > 0;
        }
};

class PluginDetectBalls : public VisionPlugin {
    protected:
        //-----------------------------
        //local copies of the vartypes tree for better performance
        //these are updated automatically if a change is reported by vartypes
        VarNotifier vnotify;
        bool filter_ball_in_field;
        double filter_ball_on_field_filter_threshold;
        bool filter_ball_in_goal;
        bool filter_ball_histogram;
        double min_greenness;
        double max_markeryness;
        bool filter_gauss;
        int exp_area_min;
        int exp_area_max;
        double exp_area_var;
        double z_height;
        bool near_robot_filter;
        double near_robot_dist_sq;
        int max_balls;
        //-----------------------------





        LUT3D* _lut;
        PluginDetectBallsSettings* _settings;
        bool _have_local_settings;
        int color_id_orange;
        int color_id_pink;
        int color_id_yellow;
        int color_id_field;

        CMVision::Histogram* histogram;

        CMVision::RegionFilter filter;

        const CameraParameters &camera_parameters;
        const RoboCupField &field;

        FieldFilter field_filter;

        bool checkHistogram(const Image<raw8>* image, const CMVision::Region* reg, double min_greenness = 0.5,
                double max_markeryness = 2.0);

    public:
        PluginDetectBalls(FrameBuffer* _buffer, LUT3D* lut, const CameraParameters &camera_params,
                const RoboCupField &field, PluginDetectBallsSettings* _settings = 0);

        ~PluginDetectBalls();

        virtual ProcessResult process(FrameData* data, RenderOptions* options);
        virtual VarList* getSettings();
        virtual string getName();
};

#endif
