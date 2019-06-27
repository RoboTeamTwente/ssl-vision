//
// Created by thijs on 27-6-19.
//

#include "plugin_techincal_challenge.h"
#include "plugin_detect_balls_settings.h"
#include "plugin_detect_balls.h"

PluginTechnicalChallenge::PluginTechnicalChallenge(FrameBuffer* _buffer, LUT3D* lut,
        const CameraParameters &camera_params, const RoboCupField &field, PluginDetectBallsSettings* _settings)
        :PluginDetectBalls(_buffer, lut, camera_params, field, _settings) {
}

PluginTechnicalChallenge::~PluginTechnicalChallenge() {
    delete histogram;
}

ProcessResult PluginTechnicalChallenge::process(FrameData* data, RenderOptions* options) {
    (void) options;
    if (data == 0) return ProcessingFailed;

    SSL_DetectionFrame* detection_frame = 0;

    detection_frame = (SSL_DetectionFrame*) data->map.get("ssl_detection_frame");
    if (detection_frame == 0)
        detection_frame = (SSL_DetectionFrame*) data->map.insert("ssl_detection_frame", new SSL_DetectionFrame());

    int color_id_ball = _lut->getChannelID(_settings->_color_label->getString());
    if (color_id_ball == - 1) {
        printf("Unknown Ball Detection Color Label: '%s'\nAborting Plugin!\n",
                _settings->_color_label->getString().c_str());
        return ProcessingFailed;
    }

    //delete any previous detection results:
    detection_frame->clear_balls();

    //TODO: add a vartype notifier for better performance.

    //initialize filter:
    if (vnotify.hasChanged()) {
        max_balls = _settings->_max_balls->getInt();
        filter.setWidth(_settings->_ball_min_width->getInt(), _settings->_ball_max_width->getInt());
        filter.setHeight(_settings->_ball_min_height->getInt(), _settings->_ball_max_height->getInt());
        filter.setArea(_settings->_ball_min_area->getInt(), _settings->_ball_max_area->getInt());
        field_filter.update(field);

        //copy all vartypes to local variables for faster repeated lookup:
        filter_ball_in_field = _settings->_ball_on_field_filter->getBool();
        filter_ball_on_field_filter_threshold = _settings->_ball_on_field_filter_threshold->getDouble();
        filter_ball_in_goal = _settings->_ball_in_goal_filter->getBool();
        filter_ball_histogram = _settings->_ball_histogram_enabled->getBool();
        if (filter_ball_histogram) {
            if (color_id_ball != color_id_orange) {
                printf("Warning: ball histogram check is only configured for orange balls!\n");
                printf("Please disable the histogram check in the Ball Detection Plugin settings\n");
            }
            if (color_id_pink == - 1 || color_id_orange == - 1 || color_id_yellow == - 1 || color_id_field == - 1) {
                printf("WARNING: some LUT color labels where undefined for the ball detection plugin\n");
                printf("         Disabling histogram check!\n");
                filter_ball_histogram = false;
            }
        }

        min_greenness = _settings->_ball_histogram_min_greenness->getDouble();
        max_markeryness = _settings->_ball_histogram_max_markeryness->getDouble();

        //setup values used for the gaussian confidence measurement:
        filter_gauss = _settings->_ball_gauss_enabled->getBool();
        exp_area_min = _settings->_ball_gauss_min->getInt();
        exp_area_max = _settings->_ball_gauss_max->getInt();
        exp_area_var = sq(_settings->_ball_gauss_stddev->getDouble());
        z_height = _settings->_ball_z_height->getDouble();

        near_robot_filter = _settings->_ball_too_near_robot_enabled->getBool();
        near_robot_dist_sq = sq(_settings->_ball_too_near_robot_dist->getDouble());
    }

    const CMVision::Region* reg = 0;

    //acquire orange region list from data-map:
    CMVision::ColorRegionList* colorlist;
    colorlist = (CMVision::ColorRegionList*) data->map.get("cmv_colorlist");
    if (colorlist == 0) {
        printf("error in ball detection plugin: no region-lists were found!\n");
        return ProcessingFailed;
    }
    reg = colorlist->getRegionList(color_id_ball).getInitialElement();

    //acquire color-labeled image from data-map:
    const Image<raw8>* image = (Image<raw8>*) (data->map.get("cmv_threshold"));
    if (image == 0) {
        printf("error in ball detection plugin: no color-thresholded image was found!\n");
        return ProcessingFailed;
    }

    int robots_blue_n = 0;
    int robots_yellow_n = 0;
    bool use_near_robot_filter = near_robot_filter;
    if (use_near_robot_filter) {
        SSL_DetectionFrame* detection_frame = (SSL_DetectionFrame*) data->map.get("ssl_detection_frame");
        if (detection_frame == 0) {
            use_near_robot_filter = false;
        }
        else {
            robots_blue_n = detection_frame->robots_blue_size();
            robots_yellow_n = detection_frame->robots_yellow_size();
            if (robots_blue_n == 0 && robots_yellow_n == 0) use_near_robot_filter = false;
        }
    }

    if (max_balls > 0) {
        list<BallDetectResult> result;
        filter.init(reg);

        while ((reg = filter.getNext()) != 0) {
            float conf = 1.0;

            if (filter_gauss) {
                int a = reg->area - bound(reg->area, exp_area_min, exp_area_max);
                conf = gaussian(a/exp_area_var);
            }

            // histogram check if enabled
            if (filter_ball_histogram && conf > 0.0
                    && checkHistogram(image, reg, min_greenness, max_markeryness) == false) {
                conf = 0;
            }

            // add filtered region to the region list
            if (conf > 0) {
                result.push_back(BallDetectResult(reg, conf));
            }

        }

        // sort result by confidence and output first max_balls region(s)
        result.sort();

        int num_ball = 0;
        list<BallDetectResult>::reverse_iterator it;
        for (it = result.rbegin(); it != result.rend(); it ++) {
            if (++ num_ball > max_balls) {
                break;
            }

            //update result:
            SSL_DetectionBall* ball = detection_frame->add_balls();

            ball->set_confidence(it->conf);

            vector2d pixel_pos(it->reg->cen_x, it->reg->cen_y);
            vector3d field_pos_3d;
            image2relativeToRobot(field_pos_3d, it->reg);

            ball->set_area(it->reg->area);
            ball->set_x(field_pos_3d.x);
            ball->set_y(field_pos_3d.y);
            ball->set_pixel_x(it->reg->cen_x);
            ball->set_pixel_y(it->reg->cen_y);
        }
    }

    return ProcessingOk;
}

VarList* PluginTechnicalChallenge::getSettings() {
    if (_have_local_settings) {
        return _settings->getSettings();
    }
    else {
        return 0;
    }
}

string PluginTechnicalChallenge::getName() {
    return "TechnicalChallenge";
}

void PluginTechnicalChallenge::image2relativeToRobot(GVector::vector3d<double> &rtr,
        const CMVision::Region* reg) const {

    if (! reg) {
        std::cout << "reg == nullptr??" << std::endl;
        rtr = GVector::vector3d<double>(10.0, 0.0, 0.0);
        return;
    }

//    int area = reg->area;
//    int width = reg->width();
//    int height = reg->height();
//    int size = width*height;
//    if (width <= 0 || height <= 0) {
//        rtr = GVector::vector3d<double>(10.0, 0.0, 0.0);
//        return;
//    }
//    double squareness = reg->squareness();

//    std::cout << "\nArea: " << area << "\nWidth: " << width << "\nHeight: " << height << "\nSize: " << size <<
//              "\nSquareness: " << squareness << std::endl;

    vector2d pixel_pos(reg->cen_x, reg->cen_y);
    camera_parameters.image2field(rtr, pixel_pos, z_height);
    std::cout << "\nx: " << rtr.x << "\ny: " << rtr.y << "\nz: " << rtr.z << std::endl;
}