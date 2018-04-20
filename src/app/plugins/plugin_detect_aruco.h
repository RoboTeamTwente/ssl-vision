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

public:
    plugin_detect_aruco_settings() {
        _settings = new VarList("Robot Aruco Detection");
        _settings->addChild(isEnabled = new VarBool("Enabled", false));
        _settings->addChild(marker_bits = new VarInt("Marker bits", 3, 3, 10));
        _settings->addChild(total_markers = new VarInt("Total Markers", 32,0));
        _settings->addChild(markers_per_team = new VarInt("Markers per Team", 16, 1));
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
