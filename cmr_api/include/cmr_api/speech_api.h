/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   speech_api.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   08/2019
*
* @brief  API Class for high level access of Speech-to-Text, Text-to-Speech and NLP functionalities
*/
#pragma once

#include "ros/ros.h"
#include <cmr_api/action_wrapper.h>
#include <cmr_msgs/SpeechToTextAction.h>
#include <cmr_msgs/TextToSpeechAction.h>
#include <cmr_msgs/NLP.h>
#include <cmr_msgs/ConversationTrigger.h>
#include <std_srvs/SetBool.h>


namespace cmr_api {

using SpeechToTextAction = cmr_msgs::SpeechToTextAction;
using SpeechToTextGoal = cmr_msgs::SpeechToTextGoal;
using SpeechToTextFeedback = cmr_msgs::SpeechToTextFeedback;
using SpeechToTextResult = cmr_msgs::SpeechToTextResultConstPtr;

using TextToSpeechAction = cmr_msgs::TextToSpeechAction;
using TextToSpeechGoal = cmr_msgs::TextToSpeechGoal;
using TextToSpeechFeedback = cmr_msgs::TextToSpeechFeedback;
using TextToSpeechResult = cmr_msgs::TextToSpeechResultConstPtr;
/**
 * @brief The Speech class to access the speech functionalities
 */
class Speech{

public:
  Speech(){};
  ~Speech();
  /**
   * @brief Initialize the interface (must be called before first use)
   *
   * @param nh node handle to use
   * @param async if true, do not wait for action servers
   */
  void init(ros::NodeHandle &nh, bool async = false);

private:
  /**
  * @brief The TTSClient class
  * Accesses a the Text-to-Speech Action server
  */
  class TTSClient :
      public ActionWrapper<TextToSpeechAction,TextToSpeechGoal,TextToSpeechFeedback,TextToSpeechResult>{

  public:
    /**
     * @brief Say a text
     * Only works, if robot is currently not speaking
     * @param text to say
     * @param block functions blocks until text was spoken
     * @param lock_dur duration in seconds to lock the text.
     * If this parameter is set, this specific text will not be said again for the given duration
     * @return true if text was sent to server
     */
    bool say(const std::string &text, bool block = true, unsigned int lock_dur = 0);

  private:
    map<string,ros::Time> locked_sentences_;

    bool textNotLocked(const std::string &text);

  }*tts_client_;

  /**
   * @brief The ConversationClient class to access Conversation and NLP functionalities
   */
  class ConversationClient{

  public:
    /**
     * @brief ConversationClient
     * @param nh node handle to use
     */
    ConversationClient(ros::NodeHandle &nh);

    /**
     * @brief Call the conversation service
     * @param session_id can be used to continue a conversation, necessary for eg. follow up intents
     * @return ConversationTriggerResponse Message
     */
    cmr_msgs::ConversationTriggerResponse callConversation(int session_id = 0);

    /**
     * @brief Call the NLP system
     * @param text input to the NLP system
     * @param session_id can be used to continue a conversation, necessary for eg. follow up intents
     * @return NLPResponse Message
     */
    cmr_msgs::NLPResponse callNLP(const std::string &text, int session_id = 0);
  private:
    ros::NodeHandle *nh_;
    ros::ServiceClient serv_conv_;
    ros::ServiceClient serv_nlp_;

  }*conversation_client_;

public:
  /**
   * @brief Getter for the TTS Client
   * @return Pointer to the TTS Client
   */
  TTSClient* getTTSClient(){return tts_client_;}

  /**
   * @brief Getter for the Conversation Client
   * @return Pointer to the Conversation Client
   */
  ConversationClient* getConversationClient(){return conversation_client_;}
};
}
