/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   led_colors.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   08/2019
*
* @brief  Struct to represent a color
*/
#pragma once
#include <std_msgs/ColorRGBA.h>
#include <random>
namespace color{
struct RGBAColor{
    /**
     * @brief RGBAColor default constructor initializes a random color
     */
    RGBAColor(){
        msg.r = getRandVal();
        msg.g = getRandVal();
        msg.b = getRandVal();
        msg.a = 100.0;

    };

    /**
     * @brief RGBAColor constructor to initialize a defined color
     * @param r red value
     * @param g green value
     * @param b blue value
     * @param a opacity value
     */
    RGBAColor(double r, double g, double b, double a){
        msg.r = r;
        msg.g = g;
        msg.b = b;
        msg.a = a;
    }
    double getRandVal()
    {
        std::random_device rd;
        std::default_random_engine re(rd());
        std::uniform_real_distribution<double> unif(0.,255.);
        return unif(re);
    }
    std_msgs::ColorRGBA toROSMsg() const {return msg;}
    std_msgs::ColorRGBA msg;
};

const static RGBAColor RED = RGBAColor(255.0,0.0,0.0,100.0);
const static RGBAColor INDIANRED = RGBAColor(205.0,92.0,92.0,80.0);
const static RGBAColor ORANGE = RGBAColor(255.0,165.0,0.0,100.0);
const static RGBAColor YELLOW = RGBAColor(255.0,255.0,0.0,100.0);
const static RGBAColor GREEN = RGBAColor(0.0,255.0,0.0,100.0);
const static RGBAColor LIGHTGREEN = RGBAColor(144.0,238.0,144.0,100.0);
const static RGBAColor BLUE = RGBAColor(0.0,0.0,255.0,100.0);
const static RGBAColor LIGHTSKYBLUE = RGBAColor(135.0,206.0,205.0,100);
const static RGBAColor VIOLET = RGBAColor(138.0,43.0,226.0,70.0);
const static RGBAColor WHITE = RGBAColor(255.0,255.0,255.0,100.0);
const static RGBAColor BLACK = RGBAColor(0.0,0.0,0.0,0.0);
const static RGBAColor GREY = RGBAColor(128.0,128.0,128.0,100.0);
const static RGBAColor BROWN = RGBAColor(165.0,42.0,42.0,100.0);

}
