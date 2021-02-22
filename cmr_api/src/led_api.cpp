#include "cmr_api/led_api.h"

cmr_api::LEDAPI::~LEDAPI()
{
  strips_->setColor(color::RGBAColor(0.0,0.0,0.0,0.0));
}

void cmr_api::LEDAPI::init(ros::NodeHandle &nh)
{
  strips_ = std::make_shared<Strips>(nh);
  panel_ = std::make_shared<Panel>(nh);
}

cmr_api::LEDAPI::Panel::Panel(ros::NodeHandle &nh)
{
  nh_ = &nh;
  servExpression_ = nh_->serviceClient<cmr_msgs::SetLEDExpression>("/sobi/led_panel/expression");
}

bool cmr_api::LEDAPI::Panel::setExpression(unsigned int expression, double duration)
{
  if(!servExpression_.exists()){
    ROS_ERROR("LED Expression service does not exist");
    return false;
  }
  cmr_msgs::SetLEDExpression setled;
  setled.request.expression.expression = expression;
  setled.request.duration = duration;
  return servExpression_.call(setled);
}

cmr_api::LEDAPI::Strips::Strips(ros::NodeHandle &nh)
{
  nh_ = &nh;
  timer_repub_ = nh_->createTimer(dur_timer_repub_, &Strips::timerRepubCallback, this);
  pub_strips_ = nh_->advertise<cmr_msgs::LEDStrips>("/sobi/led_strips",100);
  sub_speech_led_ = nh.subscribe("/sobi/speech/status_led",10,&Strips::subSpeechLedCallback,this);
  sub_speech_think_ = nh.subscribe("/sobi/speech/status_led_think",10,&Strips::subSpeechThinkCallback,this);
}

void cmr_api::LEDAPI::Strips::setColor(const color::RGBAColor &color, int limbnum, int mode)
{
  setColor(color.toROSMsg(),limbnum,mode);
}

void cmr_api::LEDAPI::Strips::setColor(const std_msgs::ColorRGBA &color, int limbnum, int mode)
{
  if(mode_ == NORMAL){
    limbnum_ = limbnum;
    last_color_msg_ = color;
  }
}

void cmr_api::LEDAPI::Strips::setColorTemp(const color::RGBAColor &color, double duration, int limbnum){
  setColorTemp(color.toROSMsg(), duration, limbnum);
}

void cmr_api::LEDAPI::Strips::setColorTemp(const std_msgs::ColorRGBA &color, double duration, int limbnum){
  if(mode_ == NORMAL){
    continuous_.step = 0;
    continuous_.total_steps = int(duration/dur_timer_repub_.toSec());
    last_color_msg_ = color;
    limbnum_ = limbnum;
    mode_ = NORMAL_TEMPORARY;
  }
}

void cmr_api::LEDAPI::Strips::blinkColor(const color::RGBAColor &color, double duration, double duration_blink, int limbnum){
  blinkColor(color.toROSMsg(), duration, duration_blink, limbnum);
}

void cmr_api::LEDAPI::Strips::blinkColor(const std_msgs::ColorRGBA &color, double duration, double duration_blink, int limbnum){
  if(mode_ == NORMAL && duration >= dur_timer_repub_.toSec() && duration_blink >= dur_timer_repub_.toSec()){
    blink_color_.color = color;
    blink_color_.step = 0;
    blink_color_.step_change = int(duration_blink/dur_timer_repub_.toSec());
    blink_color_.total_steps = int(duration/dur_timer_repub_.toSec());
    blink_color_.led_on = false;
    limbnum_ = limbnum;
    mode_ = BLINK;
  }
}

void cmr_api::LEDAPI::Strips::fadeBrightness(const color::RGBAColor &color, double duration, double duration_fade, int limbnum ){
  fadeBrightness(color.toROSMsg(), duration, duration_fade, limbnum);
}

void cmr_api::LEDAPI::Strips::fadeBrightness(const std_msgs::ColorRGBA &color, double duration, double duration_fade, int limbnum){
  if(mode_ == NORMAL){
    fade_brightness_.color = color;
    fade_brightness_.total_steps = int(duration_fade/dur_timer_repub_.toSec());
    fade_brightness_.total_repeats = int(1.0+(duration/(dur_timer_repub_.toSec()*fade_brightness_.total_steps)));
    fade_brightness_.step = 0;
    fade_brightness_.repeats = 0;
    limbnum_ = limbnum;
    mode_ = BRIGHTNESS_FADING;
  }
}

void cmr_api::LEDAPI::Strips::fadeToColor(const color::RGBAColor &color, double duration, int limbnum)
{
  fadeToColor(color.toROSMsg(), duration, limbnum);
}

void cmr_api::LEDAPI::Strips::fadeToColor(const std_msgs::ColorRGBA &color, double duration, int limbnum)
{
  if(mode_ == NORMAL){

    fade_color_.color_start = last_color_msg_;
    fade_color_.color_end = color;
    limbnum_ = limbnum;
    //Calculate the number of timer executions for fade
    fade_color_.total_steps = int(duration/dur_timer_repub_.toSec());
    fade_color_.step = 0;
    mode_ = COLOR_FADING;
  }
}

void cmr_api::LEDAPI::Strips::publishIfNew(const cmr_msgs::LEDStrips &msg)
{
  if(msg.mode != old_msg_.mode
     || msg.rgba.r != old_msg_.rgba.r
     || msg.rgba.g != old_msg_.rgba.g
     || msg.rgba.b != old_msg_.rgba.b
     || msg.rgba.a != old_msg_.rgba.a
     || msg.strip_num.num != old_msg_.strip_num.num){
    pub_strips_.publish(msg);
    old_msg_ = msg;
  }
}

void cmr_api::LEDAPI::Strips::timerRepubCallback(const ros::TimerEvent &event){

  cmr_msgs::LEDStrips strips;

  if(mode_ == NORMAL_TEMPORARY){
    continuous_.step++;
    if (continuous_.step == continuous_.total_steps) {
      mode_ = FROM_SPEECH_INTERFACE;
      last_color_msg_ = color::RGBAColor(0.0,0.0,0.0,0.0).toROSMsg();
    }
  }

  else if(mode_ == COLOR_FADING){
    //Interpolate between start and end color
    auto interp = [=](int s, int e, double ratio){
      return s + ratio * (e - s);
    };
    double ratio = fade_color_.getRatio();
    last_color_msg_.r = interp(fade_color_.color_start.r, fade_color_.color_end.r, ratio);
    last_color_msg_.g = interp(fade_color_.color_start.g, fade_color_.color_end.g, ratio);
    last_color_msg_.b = interp(fade_color_.color_start.b, fade_color_.color_end.b, ratio);
    last_color_msg_.a = interp(fade_color_.color_start.a, fade_color_.color_end.a, ratio);
    fade_color_.step++;
    if (fade_color_.step == fade_color_.total_steps) {
      mode_ = FROM_SPEECH_INTERFACE;
      last_color_msg_ = color::RGBAColor(0.0,0.0,0.0,0.0).toROSMsg();
    }
  }

  else if(mode_ == BRIGHTNESS_FADING){
    auto interp = [=](int s, int e, double ratio){
      return s + ratio * (e - s);
    };
    double ratio = fade_brightness_.getRatio();
    last_color_msg_ = fade_brightness_.color;
    last_color_msg_.a = interp(10, fade_brightness_.color.a, ratio);
    fade_brightness_.step++;
    if (fade_brightness_.step == fade_brightness_.total_steps){
      fade_brightness_.repeats++;
      fade_brightness_.step = 0;
      if(fade_brightness_.repeats == fade_brightness_.total_repeats){
        mode_ = FROM_SPEECH_INTERFACE;
        last_color_msg_ = color::RGBAColor(0.0,0.0,0.0,0.0).toROSMsg();
      }
    }
  }

  else if(mode_ == BLINK){
    if(blink_color_.step % blink_color_.step_change == 0){
      if(blink_color_.led_on){
        last_color_msg_ = color::RGBAColor(0.0,0.0,0.0,0.0).toROSMsg();
        blink_color_.led_on = false;
      }
      else{
        last_color_msg_ = blink_color_.color;
        blink_color_.led_on = true;
      }
    }
    blink_color_.step++;
    if(blink_color_.step == blink_color_.total_steps){
      last_color_msg_ = color::RGBAColor(0.0,0.0,0.0,0.0).toROSMsg();
      mode_ = FROM_SPEECH_INTERFACE;
    }
  }


  strips.rgba = last_color_msg_;
  //TODO: make this dynamically changeable
  strips.strip_num.num = limbnum_;
  strips.mode = cmr_msgs::LEDStrips::MODE_CONSTANT;
  pub_strips_.publish(strips);
}

void cmr_api::LEDAPI::Strips::subSpeechLedCallback(const std_msgs::ColorRGBA &color)
{
  if(mode_ == FROM_SPEECH_INTERFACE){
    std_msgs::ColorRGBA msg = color;
    msg.r *= 255;
    msg.g *= 255;
    msg.b *= 255;
    msg.a *= 255;
    last_color_msg_ = msg;
  }
}
void cmr_api::LEDAPI::Strips::subSpeechThinkCallback(const std_msgs::Bool &msg){
  if(mode_ == FROM_SPEECH_INTERFACE){
    std_msgs::ColorRGBA color;
    if(!msg.data){
      last_color_msg_ = color;
    }
  }
}


