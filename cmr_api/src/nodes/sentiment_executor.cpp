/* *****************************************************************
 *
 * Sentiment Executor module
 *
 * Copyright (c) 2020,
 * Institute of Mechatronic Systems,
 * Leibniz Universitaet Hannover.
 * (BSD License)
 * All rights reserved.
 *
 * http://www.imes.uni-hannover.de
 *
 * This software is distributed WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.
 *
 * For further information see http://www.linfo.org/bsdlicense.html
 *
 ******************************************************************/

/**
 * @file   sentiment_executor.cpp
 * @author Konrad Westermann (konrad.westermann@gmail.com)
 * @date   05.2020
 *
 * @brief Sentiment Executor for the visualization of emotions and internal robot states
 *
 * Depending on subscribed sentiment values, the module generates emotions and states
 * via the control of the LED panel and the moving ears including LED strips.
 */

#include "cmr_api/sentiment_executor.h"

//########## CONSTRUCTOR ###############################################################################################
SentimentExecutor::SentimentExecutor(ros::NodeHandle &node_handle):
  node_(&node_handle)
{

  ears_.init(*node_);
  led_.init(*node_);

  // === SUBSCRIBERS ===
  sub_sentiment_ = node_->subscribe("/sobi/speech/sentiment", 10, &SentimentExecutor::subSentimentCallback, this);

  // === TIMER ===
  timer_state_ = node_->createTimer(ros::Duration(0.1), &SentimentExecutor::timerCallback, this);

}

//########## CALLBACK: SUBSCRIBER ######################################################################################
void SentimentExecutor::subSentimentCallback(const cmr_msgs::Sentiment &msg)
{
  hold_sentiment_.set_value(msg);
}

//########## CALLBACK: TIMER ###########################################################################################
void SentimentExecutor::timerCallback(const ros::TimerEvent &evt)
{
  if(hold_sentiment_.has_new_value()){

    cmr_msgs::Sentiment sentiment;
    hold_sentiment_.get_value(sentiment);

    settingsLED setLED;
    settingsWiggle setWiggle;
    settingsHang setHang;
    settingsGoTo setGoTo;

    double speak_time = sentiment.output_text.length()*SEC_PER_CHAR;
    //Set time for expressions to at least 2.5 seconds
    if(speak_time < 2.5) speak_time = 2.5;

    //speak_time = 5; //Testing

    //robot states
    if(sentiment.mode == "state_normal" && sentiment.sentiment == 0.0){
      speak_time = 1;
      setLED = {true, NEUTRAL, NORMAL, EARS, color::BLACK};
      setGoTo = {true, EARS, 0};
    }
    else if(sentiment.mode == "state_normal" && sentiment.sentiment == 1.0){
      speak_time = 1;
      setLED = {true, NEUTRAL, NORMAL, EARS, color::BLACK};
    }
    else if(sentiment.mode == "state_user_input"){
      speak_time = state_time;
      setLED = {true, CURIOUS, BRIGHTNESS_FADING, EARS, color::GREEN, 2.0};
      setGoTo = {true, EARS, -90};
    }
    else if(sentiment.mode == "state_process"){
      //speak_time = state_time;
      setLED = {true, PROCESS, BRIGHTNESS_FADING, EARS, color::ORANGE, 2.0};
    }
    else if(sentiment.mode == "state_error"){
      //speak_time = state_time;
      setLED = {true, ERROR, BLINK, EARS, color::RED, 0.5};
    }
    else if(sentiment.mode == "state_tired"){
      //speak_time = state_time;
      setLED = {true, SLEEP, NORMAL, EARS, color::BLACK};
    }

    // robot emotions
    else if(sentiment.mode == "sentiment" && sentiment.sentiment < -0.5){
      setLED = {true, SAD, BRIGHTNESS_FADING, EARS, color::VIOLET, 1.5};
      setHang = {true, EARS, 110, 3.0};
    }
    else if(sentiment.mode == "sentiment" && sentiment.sentiment < 0.0){
      setLED = {true, SAD, NORMAL_TEMPORARY, EARS, color::VIOLET};
      setHang = {true, EARS, 30, 2.0};
    }
    else if(sentiment.mode == "sentiment" && sentiment.sentiment > 0.5){
      setLED = {true, LAUGHING, BRIGHTNESS_FADING, EARS, color::YELLOW, 1.0};
      setWiggle = {true, EARS, 20, 0.3};
    }
    else if(sentiment.mode == "sentiment" && sentiment.sentiment > 0.0){
      setLED = {true, LAUGHING, NORMAL_TEMPORARY, EARS, color::ORANGE};
      setWiggle = {true, EARS, 45, 0.5};
    }
    else if(sentiment.mode == "emotion_confused"){
      setLED = {true, CONFUSED, NORMAL_TEMPORARY, EARS, color::INDIANRED};
      setWiggle = {true, EARS, 110, 1.0, 1.0};
    }
    else if(sentiment.mode == "emotion_curious"){
      setLED = {true, CURIOUS, BRIGHTNESS_FADING, EARS, color::GREEN, 2.0};
      setHang = {true, EARS, -90};
    }
    else if(sentiment.mode == "emotion_angry"){
      setLED = {true, ANGRY, BRIGHTNESS_FADING, EARS, color::RED, 2.0};
      setHang = {true, EARS, -30, 0.1};
    }


    if(setLED.set){
      led_.getStrips()->setMode(cmr_api::LEDAPI::StripsMode::NORMAL);
      led_.getPanel()->setExpression(setLED.expression,speak_time);
    }
      if(setLED.mode == NORMAL){
        led_.getStrips()->setColor(setLED.color, setLED.limbNum);
        led_.getStrips()->setMode(cmr_api::LEDAPI::StripsMode::FROM_SPEECH_INTERFACE);
      }
      else if(setLED.mode == NORMAL_TEMPORARY){
        led_.getStrips()->setColorTemp(setLED.color, speak_time, setLED.limbNum);
      }
      else if(setLED.mode == BLINK){
        led_.getStrips()->blinkColor(setLED.color, speak_time, setLED.spec_duration, setLED.limbNum);
      }
      else if(setLED.mode == COLOR_FADING){
        led_.getStrips()->fadeToColor(setLED.color, speak_time, setLED.limbNum);
      }
      else if(setLED.mode == BRIGHTNESS_FADING){
        led_.getStrips()->fadeBrightness(setLED.color, speak_time, setLED.spec_duration, setLED.limbNum);
      }
      else led_.getStrips()->setMode(cmr_api::LEDAPI::StripsMode::FROM_SPEECH_INTERFACE);

    if(setHang.set){
      if(speak_time <= 2 * setHang.dur_move) setHang.dur_move = (speak_time / 2) - 0.1;
      ears_.motionHang(speak_time, setHang.dur_move, setHang.limbNum, setHang.angle);
    }
    else if(setGoTo.set){
      ears_.motionGoTo(setGoTo.dur_move, setGoTo.limbNum, setGoTo.angle);
    }
    else if(setWiggle.set){
      ears_.motionWiggle(speak_time, setWiggle.limbNum, setWiggle.angle, setWiggle.timePerRepeat, setWiggle.timeToStay); //TODO: repeats berechnet aus den beiden Zeiten!
    }
  }
}

//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sentiment_executor");

  ros::NodeHandle node_handle;
  SentimentExecutor cmr_api_node(node_handle);

  ROS_INFO("Sentiment executor started...");
  ros::spin();

  return 0;
}
