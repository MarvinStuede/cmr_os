#include "cmr_api/speech_api.h"

cmr_api::Speech::~Speech()
{
  delete tts_client_;
  delete conversation_client_;

}

void cmr_api::Speech::init(ros::NodeHandle &nh, bool async)
{
  tts_client_ = new TTSClient();
  conversation_client_ = new ConversationClient(nh);
  tts_client_->init("/sobi/speech/tts", async);
}

bool cmr_api::Speech::TTSClient::say(const std::string &text, bool block, unsigned int lock_dur)
{
  TextToSpeechGoal goal;
  goal.text_input = text;

  //If text is in lock list, do not say the text
  if(!textNotLocked(text)) return false;

  if(sendGoal(goal)){

    if(lock_dur > 0){
      //Add text to locked list, with time of release
      locked_sentences_[text] = ros::Time::now() + ros::Duration(float(lock_dur));
    }
    if(!block) return true;
    waitForCurrentAction();
    return true;
  }

  else{
    ROS_ERROR_STREAM("TTS Action with text \'" <<text<< "\' not sent!");
  }
  return false;
}

bool cmr_api::Speech::TTSClient::textNotLocked(const string &text)
{
  if (locked_sentences_.find(text) == locked_sentences_.end()) return true;
  if ((ros::Time::now() - locked_sentences_[text]).toSec() > 0.0) {
    locked_sentences_.erase(text);
    return true;
  }
  return false;
}

cmr_api::Speech::ConversationClient::ConversationClient(ros::NodeHandle &nh)
{
  nh_ = &nh;
  serv_conv_ = nh_->serviceClient<cmr_msgs::ConversationTrigger>("/sobi/speech/conversation");
  serv_nlp_ = nh_->serviceClient<cmr_msgs::NLP>("/sobi/speech/nlp");
}

cmr_msgs::ConversationTriggerResponse cmr_api::Speech::ConversationClient::callConversation(int session_id)
{
  cmr_msgs::ConversationTrigger sb;
  sb.request.data = true;
  sb.request.session_id = session_id;
  serv_conv_.call(sb);
  return sb.response;
}

cmr_msgs::NLPResponse cmr_api::Speech::ConversationClient::callNLP(const std::string &text, int session_id)
{
  cmr_msgs::NLP nlp;
  nlp.request.input_text = text;
  nlp.request.session_id = session_id;
  serv_nlp_.call(nlp);
  return nlp.response;
}
