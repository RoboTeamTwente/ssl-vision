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
  \file    cmvision_threshold.h
  \brief   C++ Interface: cmvision_threshold
  \author  James Bruce (Original CMVision implementation and algorithms),
           Some code restructuring, and data structure changes: Stefan Zickler 2008
*/
//========================================================================
#ifndef CMVISIONTHRESHOLD_H
#define CMVISIONTHRESHOLD_H

#include "lut3d.h"
#include "image_interface.h"
#include "image.h"
#include "colors.h"
#include "timer.h"
#include <thread>

/**
	@author James Bruce (Original CMVision implementation and algorithms),
          Some code restructuring, and data structure changes: Stefan Zickler 2008
*/
class CMVisionThreshold{
protected:
    static unsigned int thread_count;
public:
    CMVisionThreshold();

    ~CMVisionThreshold();

    static bool setThreads(unsigned int threads) {
        if (threads > 128 || threads < 1) {
            return false;
        }
        thread_count = threads;
        return true;
    }

    static bool thresholdImageYUV422_UYVY(Image<raw8> * target, const RawImage * source, YUVLUT * lut);
    static bool thresholdImageYUV444(Image<raw8> * target, const ImageInterface * source, YUVLUT * lut);
    static bool thresholdImageRGB(Image<raw8> * target, const ImageInterface * source, RGBLUT * lut);
    static void thresholdImageYUV422_UYVYWorker(lut_mask_t * LUT, int X_SHIFT, int Y_SHIFT, int Z_SHIFT, int Z_AND_Y_BITS, int Z_BITS, uyvy* source_pointer, raw8* target_pointer, unsigned int start, unsigned int end);

    static void colorizeImageFromThresholding(rgbImage & target, const Image<raw8> & source, LUT3D * lut);

    //static void thresholdImage(Image * target, const Image<yuv> * source, const YUVLUT * lut);

    //static void thresholdImage(Image * target, const Image<yuvy> * source, const YUVLUT * lut);

    //static void thresholdImage(Image * target, const Image<uyvy> * source, const LUT3D * lut);

    static void thresholdImageYUV444Worker(lut_mask_t *LUT, int X_SHIFT, int Y_SHIFT, int Z_SHIFT, int Z_AND_Y_BITS, int Z_BITS,
                                    yuv *source_pointer, raw8 *target_pointer, unsigned int start, unsigned int end);

    static void thresholdImageRGBWorker(lut_mask_t *LUT, int X_SHIFT, int Y_SHIFT, int Z_SHIFT, int Z_AND_Y_BITS, int Z_BITS,
                                 rgb *source_pointer, raw8 *target_pointer, unsigned int start, unsigned int end);
};

#endif
