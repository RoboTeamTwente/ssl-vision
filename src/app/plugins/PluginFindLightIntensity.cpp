//
// Created by thijs on 20-2-19.
//

#include "PluginFindLightIntensity.h"

VarList *PluginFindLightIntensity::getSettings() {
    return _settings->getSettings();
}

PluginFindLightIntensity::PluginFindLightIntensity(FrameBuffer* _buffer, const CameraParameters &camera_params)
    : VisionPlugin(_buffer), camera_parameters(camera_params) {
    print = 0;
    _settings = new PluginFindLightIntensitySettings();
    _notifier.addRecursive(_settings->getSettings());
    _is_enabled = _settings->is_enabled->getBool();
    _left_lower_x = static_cast<unsigned int>(_settings->left_lower_x->getInt());
    _left_upper_x = static_cast<unsigned int>(_settings->left_upper_x->getInt());
    _left_lower_y = static_cast<unsigned int>(_settings->left_lower_y->getInt());
    _left_upper_y = static_cast<unsigned int>(_settings->left_upper_y->getInt());
    _right_lower_x = static_cast<unsigned int>(_settings->right_lower_x->getInt());
    _right_upper_x = static_cast<unsigned int>(_settings->right_upper_x->getInt());
    _right_lower_y = static_cast<unsigned int>(_settings->right_lower_y->getInt());
    _right_upper_y = static_cast<unsigned int>(_settings->right_upper_y->getInt());
}

unsigned char PluginFindLightIntensity::getTotalBrightnessInArea(FrameData* data, int lowerX, int upperX, int lowerY,
        int upperY) {

    unsigned char* image = data->video.getData();
    unsigned char* pixel;
    int width = data->video.getWidth();
    int height = data->video.getHeight();

//    static bool firstTime = true;
//    if (firstTime) {
//        firstTime = false;
//        std::cout << "x: " << width << ", y: " << height << std::endl;
//    }

    //std::cout << "color:" << data->video.getColorFormat() << std::endl;
    if (lowerX < 0 || lowerX >= upperX || lowerX >= width) {
        std::cerr << "lowerX is invalid" << std::endl;
        return 0;
    }
    if (lowerY < 0 || lowerY >= upperY || lowerY >= height) {
        std::cerr << "lowerY is invalid" << std::endl;
        return 0;
    }
    if (upperX < 0 || lowerX >= width) {
        std::cerr << "upperX is invalid" << std::endl;
        return 0;
    }
    if (upperY < 0 || upperY >= height) {
        std::cerr << "upperY is invalid" << std::endl;
        return 0;
    }

    char edgeSize = 2;
    unsigned int nPixels = 0;
    short pixelColor;
    unsigned long long totalPixelColor = 0;
    for (int iX = lowerX; iX < upperX; iX ++) {
        for (int iY = lowerY; iY < upperY; iY ++) {
            pixelColor = 0;
            pixel = image + iX*3 + iY*3*width;
            for (int iC = 0; iC < 3; iC ++) {
                pixelColor += *(pixel + iC);
                nPixels ++;
                if (iX < lowerX + edgeSize || iX >= upperX - edgeSize || iY < lowerY + edgeSize
                        || iY >= upperY - edgeSize)
                    *(pixel + iC) = 0;
            }
            totalPixelColor += pixelColor;
        }
    }

    return static_cast<unsigned char>((nPixels > 0) ? totalPixelColor / nPixels : 0);
}

ProcessResult PluginFindLightIntensity::process(FrameData* data, RenderOptions* options) {
    (void)options;

    //if (_notifier.hasChanged()) {
        _is_enabled = _settings->is_enabled->getBool();
        _left_lower_x = static_cast<unsigned int>(_settings->left_lower_x->getInt());
        _left_upper_x = static_cast<unsigned int>(_settings->left_upper_x->getInt());
        _left_lower_y = static_cast<unsigned int>(_settings->left_lower_y->getInt());
        _left_upper_y = static_cast<unsigned int>(_settings->left_upper_y->getInt());
        _right_lower_x = static_cast<unsigned int>(_settings->right_lower_x->getInt());
        _right_upper_x = static_cast<unsigned int>(_settings->right_upper_x->getInt());
        _right_lower_y = static_cast<unsigned int>(_settings->right_lower_y->getInt());
        _right_upper_y = static_cast<unsigned int>(_settings->right_upper_y->getInt());
    //}
    if (!_is_enabled) {
        return ProcessingOk;
    }

    unsigned char totalColorLeft = getTotalBrightnessInArea(
            data,
            _left_lower_x,
            _left_upper_x,
            _left_lower_y,
            _left_upper_y);
    unsigned char totalColorRight = getTotalBrightnessInArea(
            data,
            _right_lower_x,
            _right_upper_x,
            _right_lower_y,
            _right_upper_y);
    
    if (print++ < 30) {
        return ProcessingOk;
    }
    print = 0;
    std::cout << "        Left square total brightness: " << static_cast<unsigned short>(totalColorLeft) << std::endl;
    std::cout << "Right square total brightness: " << static_cast<unsigned short>(totalColorRight) << std::endl;

    return ProcessingOk;
}

void PluginFindLightIntensity::postProcess(FrameData* data, RenderOptions* options) {
    VisionPlugin::postProcess(data, options);
}

std::string PluginFindLightIntensity::getName() {
    return "FindLightIntensity";
}
