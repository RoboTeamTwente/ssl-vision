//
// Created by thijs on 20-2-19.
//

#include "PluginFindLightIntensity.h"

unsigned long long PluginFindLightIntensity::getTotalBrightnessInArea(FrameData *data, int lowerX, int upperX, int lowerY, int upperY) {
    unsigned char *image = data->video.getData();
    unsigned char *pixel;
    int width = data->video.getWidth();
    int height = data->video.getHeight();

    if (lowerX < 0 || lowerX > upperX || lowerX > width) {
        std::cerr << "lowerX is invalid" << std::endl;
        return 0;
    }
    if (lowerY < 0 || lowerY > upperY || lowerY > height) {
        std::cerr << "lowerY is invalid" << std::endl;
        return 0;
    }
    if (upperX < 0 || lowerX > width) {
        std::cerr << "upperX is invalid" << std::endl;
        return 0;
    }
    if (upperY < 0 || upperY > width) {
        std::cerr << "upperY is invalid" << std::endl;
        return 0;
    }

    short pixelColor;
    unsigned long long totalPixelColor = 0;
    for (int iX = lowerX; iX < upperX; iX++) {
        for (int iY = lowerY; iY < upperY; iY++) {
            pixel = image + iX * 3 + iY * 3 *width;
            pixelColor = *pixel + *(pixel+1) + *(pixel+2);
            totalPixelColor += pixelColor;
        }
    }
    return totalPixelColor;
}

ProcessResult PluginFindLightIntensity::process(FrameData *data, RenderOptions * options) {

unsigned long long totalColor = getTotalBrightnessInArea(data, 10, 100, 10, 100);

std::cout << "total brightness: " << totalColor << std::endl;

return ProcessingOk;
}

void PluginFindLightIntensity::postProcess(FrameData* data, RenderOptions* options) {
    VisionPlugin::postProcess(data, options);
}

std::string PluginFindLightIntensity::getName() {
    return "FindLightIntensity";
}
