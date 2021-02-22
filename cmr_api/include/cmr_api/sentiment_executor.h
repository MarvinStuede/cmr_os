/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   sentiment_executor.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @author Konrad Westermann (Konrad.Westermann@gmail.com)
 * @date   05/2020
 *
 * @brief Sentiment Executor for the visualization of emotions and internal robot states
 *
 * Depending on subscribed sentiment values, the module generates emotions and states
 * via the control of the LED panel and the moving ears including LED strips.
 */

#pragma once
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"
#include "cmr_api/ear_api.h"
#include "cmr_api/led_api.h"
#include "cmr_api/arm_api.h"
#include "cmr_msgs/Sentiment.h"
#include "cmr_os/cmr_holder.h"

//Rough extimate how long the speech api needs per character
static constexpr double SEC_PER_CHAR = 0.06;
/**
 * @brief The SentimentExecutor class
 */
class SentimentExecutor
{
public:
    /**
     * @brief SentimentExecutor
     * @param node_handle
     */
    SentimentExecutor(ros::NodeHandle &node_handle);

private:
    // node handle
    ros::NodeHandle *node_;

    // ros communication
    ros::Subscriber sub_sentiment_;
    ros::Timer timer_state_;

    cmr_api::EarAPI ears_;
    cmr_api::LEDAPI led_;
    cmr_api::ArmAPI arms_;

    cmr_os::cmrHolder<cmr_msgs::Sentiment> hold_sentiment_;

    int NEUTRAL = cmr_msgs::LEDExpression::NEUTRAL;
    int SAD = cmr_msgs::LEDExpression::SAD;
    int SLEEP = cmr_msgs::LEDExpression::SLEEP;
    int ERROR = cmr_msgs::LEDExpression::ERROR;
    int ANGRY = cmr_msgs::LEDExpression::ANGRY;
    int LAUGHING = cmr_msgs::LEDExpression::LAUGHING;
    int CONFUSED = cmr_msgs::LEDExpression::CONFUSED;
    int CURIOUS = cmr_msgs::LEDExpression::CURIOUS;
    int PROCESS = cmr_msgs::LEDExpression::PROCESS;

    int NORMAL = cmr_api::LEDAPI::StripsMode::NORMAL;
    int NORMAL_TEMPORARY = cmr_api::LEDAPI::StripsMode::NORMAL_TEMPORARY;
    int COLOR_FADING = cmr_api::LEDAPI::StripsMode::COLOR_FADING;
    int BRIGHTNESS_FADING = cmr_api::LEDAPI::StripsMode::BRIGHTNESS_FADING;
    int BLINK = cmr_api::LEDAPI::StripsMode::BLINK;
    int FROM_SPEECH_INTERFACE = cmr_api::LEDAPI::StripsMode::FROM_SPEECH_INTERFACE;

    int EARS = cmr_msgs::LimbNum::EARS;
    int EAR_LEFT = cmr_msgs::LimbNum::EAR_LEFT;
    int EAR_RIGHT = cmr_msgs::LimbNum::EAR_RIGHT;

    double state_time = 60.0;

    /**
     * @brief Struct of parameters for LED control
     */
    struct settingsLED{
      bool set = false;
      int expression;
      int mode;
      int limbNum;
      color::RGBAColor color;
      double spec_duration = 1.0;
    };

    /**
     * @brief struct of parameters for ear wiggling
     */
    struct settingsWiggle{
      bool set = false;
      int limbNum;
      double angle;
      double timePerRepeat;
      double timeToStay = 0.001;
    };

    /**
     * @brief struct of parameters for ear hanging
     */
    struct settingsHang{
      bool set = false;
      int limbNum;
      double angle;
      double dur_move = 1;
    };

    /**
     * @brief struct of parameters for permanent change of ear position
     */
    struct settingsGoTo{
      bool set = false;
      int limbNum;
      double angle;
      double dur_move = 1;
    };

    // callbacks
    void subSentimentCallback(const cmr_msgs::Sentiment &msg);
    void timerCallback(const ros::TimerEvent &evt);
};
