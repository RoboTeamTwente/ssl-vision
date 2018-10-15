//
// Created by wouter on 3/26/18.
//

#ifndef SSL_VISION_PLUGIN_DETECT_ARUCO_H
#define SSL_VISION_PLUGIN_DETECT_ARUCO_H


#include <VarNotifier.h>
#include <aruco_detector.h>
#include "visionplugin.h"
#include <camera_calibration.h>
#include <messages_robocup_ssl_detection.pb.h>
#include "opencv2/photo.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

class plugin_detect_aruco;

class RunFilter{

public:
    RunFilter(unsigned int _size, unsigned int _threshold);
    void filter(std::vector<PosRotId>& input);

private:
    unsigned int size;
    unsigned int threshold;
    std::vector<std::vector<PosRotId>> history;
    unsigned int * counters;


};

class plugin_detect_aruco_settings{
    friend class plugin_detect_aruco;

protected:

    VarList * _settings;
    VarBool * isEnabled;
    VarInt * marker_bits;
    VarInt * total_markers;
    VarInt * markers_per_team;
    VarInt * lower_blue_margin;
    VarInt * lower_green_margin;
    VarInt * lower_red_margin;
    VarInt * delta_margin;

public:
    plugin_detect_aruco_settings() {
        _settings = new VarList("Robot Aruco Detection");
        _settings->addChild(isEnabled = new VarBool("Enabled", false));
        _settings->addChild(marker_bits = new VarInt("Marker bits", 3, 3, 10));
        _settings->addChild(total_markers = new VarInt("Total Markers", 32,0));
        _settings->addChild(markers_per_team = new VarInt("Markers per Team", 16, 1));
        _settings->addChild(lower_blue_margin = new VarInt("Blue Margin",100,0,255));
        _settings->addChild(lower_green_margin = new VarInt("Green Margin",100,0,255));
        _settings->addChild(lower_red_margin = new VarInt("Red Margin",100,0,255));
        _settings->addChild(delta_margin = new VarInt("Delta Margin",10,0,31));
    }

    VarList * getSettings() {
        return _settings;
    }
};


class plugin_detect_aruco
: public VisionPlugin {
protected:

    bool _enabled;
    int _marker_bits;
    int _total_markers;
    int _markers_per_team;
    unsigned char _lower_blue_margin;
    unsigned char _lower_green_margin;
    unsigned char _lower_red_margin;
    unsigned char _delta_margin;

    VarNotifier _notifier;
    plugin_detect_aruco_settings * _settings;

    ArucoDetector * detector;
    RunFilter * filter;



    cv::Mat * graylut;
    const CameraParameters& camera_parameters;
    const RoboCupField& field;


public:
    ProcessResult process(FrameData * data, RenderOptions * options) override;
    void postProcess(FrameData * data, RenderOptions * options) override;
    VarList * getSettings() override;
    string getName() override;

    plugin_detect_aruco(FrameBuffer *_buffer, const CameraParameters &camera_params, const RoboCupField &field);

    ~plugin_detect_aruco();
};




#endif //SSL_VISION_PLUGIN_DETECT_ARUCO_H
