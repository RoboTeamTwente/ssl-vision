//
// Created by thijs on 27-6-19.
//

#ifndef SSL_VISION_PLUGIN_DETECT_BALLS_SETTINGS_H
#define SSL_VISION_PLUGIN_DETECT_BALLS_SETTINGS_H

class PluginDetectBallsSettings {
        friend class PluginDetectBalls;
        friend class PluginTechnicalChallenge;
    protected:
        VarList * _settings;

        VarInt    * _max_balls;
        VarString * _color_label;
        VarList   * _filter_general;
        VarDouble * _ball_z_height;
        VarInt    * _ball_min_width;
        VarInt    * _ball_max_width;
        VarInt    * _ball_min_height;
        VarInt    * _ball_max_height;
        VarInt    * _ball_min_area;
        VarInt    * _ball_max_area;

        VarList * _filter_gauss;
        VarBool * _ball_gauss_enabled;
        VarInt  * _ball_gauss_min;
        VarInt  * _ball_gauss_max;
        VarDouble * _ball_gauss_stddev;

        VarList   * _filter_too_near_robot;
        VarBool   * _ball_too_near_robot_enabled;
        VarDouble * _ball_too_near_robot_dist;
        VarList   * _filter_histogram;
        VarBool   * _ball_histogram_enabled;
        VarDouble * _ball_histogram_min_greenness;
        VarDouble * _ball_histogram_max_markeryness;
        VarList   * _filter_geometry;
        VarBool   * _ball_on_field_filter;
        VarDouble * _ball_on_field_filter_threshold;
        VarBool   * _ball_in_goal_filter;

    public:
        PluginDetectBallsSettings() {

            _settings=new VarList("Ball Detection");

            _settings->addChild(_max_balls = new VarInt("Max Ball Count",10));
            _settings->addChild(_color_label = new VarString("Ball Color","Orange"));

            _settings->addChild(_filter_general = new VarList("Ball Properties"));
            _filter_general->addChild(_ball_z_height = new VarDouble("Ball Z-Height", 30.0));
            _filter_general->addChild(_ball_min_width = new VarInt("Min Width (pixels)", 3));
            _filter_general->addChild(_ball_max_width = new VarInt("Max Width (pixels)", 30));
            _filter_general->addChild(_ball_min_height = new VarInt("Min Height (pixels)", 3));
            _filter_general->addChild(_ball_max_height = new VarInt("Max Height (pixels)", 30));
            _filter_general->addChild(_ball_min_area = new VarInt("Min Area (sq-pixels)", 9));
            _filter_general->addChild(_ball_max_area = new VarInt("Max Area (sq-pixels)", 1000));

            _settings->addChild(_filter_gauss = new VarList("Gaussian Size Filter"));
            _filter_gauss->addChild(_ball_gauss_enabled = new VarBool("Enable Filter",true));
            _filter_gauss->addChild(_ball_gauss_min = new VarInt("Expected Min Area (sq-pixels)", 30));
            _filter_gauss->addChild(_ball_gauss_max = new VarInt("Expected Max Area (sq-pixels)", 40));
            _filter_gauss->addChild(_ball_gauss_stddev = new VarDouble("Expected Area StdDev (sq-pixels)", 10.0));

            _settings->addChild(_filter_too_near_robot = new VarList("Near Robot Filter"));
            _filter_too_near_robot->addChild(_ball_too_near_robot_enabled = new VarBool("Enable Filter",true));
            _filter_too_near_robot->addChild(_ball_too_near_robot_dist = new VarDouble("Distance (mm)",70));

            _settings->addChild(_filter_histogram = new VarList("Histogram Filter"));
            _filter_histogram->addChild(_ball_histogram_enabled = new VarBool("Enable Filter",true));
            _filter_histogram->addChild(_ball_histogram_min_greenness = new VarDouble("Min Greenness",0.5));
            _filter_histogram->addChild(_ball_histogram_max_markeryness = new VarDouble("Max Markeryness",2.0));

            _settings->addChild(_filter_geometry = new VarList("Geometry Filters"));
            _filter_geometry->addChild(_ball_on_field_filter = new VarBool("Ball-In-Field Filter",true));
            _filter_geometry->addChild(_ball_on_field_filter_threshold = new VarDouble("Ball-In-Field Extra Space (mm)",300.0));
            _filter_geometry->addChild(_ball_in_goal_filter = new VarBool("Ball-In-Goal Filter",false));

        }
        VarList * getSettings() {
            return _settings;
        }

};


#endif //SSL_VISION_PLUGIN_DETECT_BALLS_SETTINGS_H
