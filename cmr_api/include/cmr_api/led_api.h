/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   led_api.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   08/2019
*
* @brief  API Class for high level access to the LED Strips and Panel
*/
#pragma once
#include "ros/ros.h"
#include <cmr_api/action_wrapper.h>
#include <memory>
#include <cmr_msgs/LEDExpression.h>
#include <cmr_msgs/LEDStrips.h>
#include "cmr_api/led_colors.h"
#include <cmr_msgs/SetLEDExpression.h>
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Bool.h"

using std::shared_ptr;
namespace cmr_api {

/**
 * @brief The LED class for access to the LED Strips and Panel
 */
class LEDAPI{

public:
    /**
   * @brief The StripsMode enum
   * NORMAL: Colors can be set via the setColor Methods
   * FROM_SPEECH_INTERFACE: Color is set according to speech status led topic.
   * This sets the strip to the same color as the microphone array leds.
   */
  enum StripsMode{
    NORMAL,
    NORMAL_TEMPORARY,
    COLOR_FADING,
    BRIGHTNESS_FADING,
    BLINK,
    FROM_SPEECH_INTERFACE
  };

  LEDAPI(){};
  ~LEDAPI();
  /**
   * @brief Initialize the interface (must be called before first use)
   *
   * @param nh node handle to use
   */
  void init(ros::NodeHandle &nh);

private:
  /**
   * @brief The Panel class to access the LED Panel
   */
  class Panel{
  public:
    /**
     * @brief Panel Constructor
     * @param nh node handle to use
     */
    Panel(ros::NodeHandle &nh);

    /**
     * @brief Sets an expression to the LED Panel. If an animation exists for this expression, the animation is played.
     * Otherwise the expression is shown statically. See cmr_msgs/LEDExpression for valid expression values.
     * @param expression a cmr_msgs/LEDExpression value
     * @param duration duration to show the expression
     * @return true if expression set successfully
     */
    bool setExpression(unsigned int expression, double duration = 2.0);

  private:
    ros::NodeHandle* nh_;
    ros::ServiceClient servExpression_;
  };

  class Strips{
  public:
    /**
     * @brief Strips Constructor
     * @param nh node handle to use
     */
    Strips(ros::NodeHandle &nh);
    ~Strips(){};

    /**
     * @brief Set a specific color to a strip. Only possible if the StripsMode is NORMAL
     * @param color to set
     * @param limbnum to use
     * @param mode to use. Currently CONSTANT and FADING is implemented.
     */
    void setColor(const color::RGBAColor &color, int limbnum = cmr_msgs::LimbNum::ALL, int mode = cmr_msgs::LEDStrips::MODE_NONE);
    /**
     * @brief Set a specific color to a strip. Only possible if the StripsMode is NORMAL (set with Strips::setMode)
     * @param color to set
     * @param limbnum to use
     * @param mode to use. Currently CONSTANT and FADING is implemented.
     */
    void setColor(const std_msgs::ColorRGBA &color, int limbnum = cmr_msgs::LimbNum::ALL, int mode = cmr_msgs::LEDStrips::MODE_NONE);

    void setColorTemp(const color::RGBAColor &color, double duration = 0.0, int limbnum = cmr_msgs::LimbNum::ALL);

    void setColorTemp(const std_msgs::ColorRGBA &color, double duration = 0.0, int limbnum = cmr_msgs::LimbNum::ALL);

    void fadeBrightness(const color::RGBAColor &color, double duration, double duration_fade, int limbnum = cmr_msgs::LimbNum::ALL);

    void fadeBrightness(const std_msgs::ColorRGBA &color, double duration, double duration_fade, int limbnum = cmr_msgs::LimbNum::ALL);

    void blinkColor(const color::RGBAColor &color, double duration, double duration_blink, int limbnum = cmr_msgs::LimbNum::ALL);

    void blinkColor(const std_msgs::ColorRGBA &color, double duration, double duration_blink, int limbnum = cmr_msgs::LimbNum::ALL);

    void fadeToColor(const color::RGBAColor &color, double duration, int limbnum = cmr_msgs::LimbNum::ALL);

    void fadeToColor(const std_msgs::ColorRGBA &color, double duration, int limbnum = cmr_msgs::LimbNum::ALL);

    /**
     * @brief setMode sets the current StripsMode
     * @param mode StripsMode to usw
     */
    void setMode(StripsMode mode){mode_ = mode;}

    /**
     * @brief getMode
     * @return current StripsMode
     */
    StripsMode getMode(){return mode_;}

  private:
    ros::Publisher pub_strips_;
    ros::NodeHandle* nh_;
    ros::Timer timer_repub_;
    ros::Subscriber sub_speech_led_;
    ros::Subscriber sub_speech_think_;
    StripsMode mode_ = FROM_SPEECH_INTERFACE;
    std_msgs::ColorRGBA last_color_msg_;
    u_int limbnum_ = cmr_msgs::LimbNum::EARS;
    ros::Duration dur_timer_repub_ = ros::Duration(0.1);

    struct ColorFade{
      std_msgs::ColorRGBA color_start;
      std_msgs::ColorRGBA color_end;
      ros::Time end_time;
      u_int total_steps;
      u_int step;
      u_int limbnum;
      double getRatio(){return double(step)/total_steps;}
    }fade_color_;

    struct BrightnessFade{
      std_msgs::ColorRGBA color;
      //ros::Time end_time;
      u_int total_steps;
      u_int step;
      u_int repeats;
      u_int total_repeats;
      double getRatio(){return double(step)/total_steps;}
    }fade_brightness_;

    struct ColorBlink{
      std_msgs::ColorRGBA color;
      u_int step;
      u_int step_change;
      u_int total_steps;
      bool led_on;
    }blink_color_;

    struct Continuous{
      u_int step;
      u_int total_steps;
    }continuous_;



    /**
     * @brief Publishes a cmr_msgs::LEDStrips message if it is differenct than the message sent the last time this method was called.
     * @param msg
     */
    void publishIfNew(const cmr_msgs::LEDStrips &msg);

    void subSpeechLedCallback(const std_msgs::ColorRGBA &color);
    void subSpeechThinkCallback(const std_msgs::Bool &msg);
    void timerRepubCallback(const ros::TimerEvent &event);

    cmr_msgs::LEDStrips old_msg_;
  };
  std::shared_ptr<Panel> panel_;
  std::shared_ptr<Strips> strips_;

public:
  /**
   * @brief Getter for the panel
   * @return Shared pointer to the panel
   */
  std::shared_ptr<Panel> getPanel(){return panel_;}
  /**
   * @brief Getter for the strips
   * @return Shared pointer to the strips
   */
  std::shared_ptr<Strips> getStrips(){return strips_;}

};
/**
 * @brief Convert a String to the corresponding expression integer
 * @param str
 * @return
 */
inline unsigned int stringToExpression(const string &str)
{
  cmr_msgs::LEDExpression expr;
  if(str == "SAD") return expr.SAD;
  else if(str == "NEUTRAL") return expr.NEUTRAL;
  else if(str == "ANGRY") return expr.ANGRY;
  else if(str == "ERROR") return expr.ERROR;
  else if(str == "SLEEP") return expr.SLEEP;
  else if(str == "CURIOUS") return expr.CURIOUS;
  else if(str == "PROCESS") return expr.PROCESS;
  else if(str == "CONFUSED") return expr.CONFUSED;
  else if(str == "LAUGHING") return expr.LAUGHING;
  else return expr.NEUTRAL;
}
}
