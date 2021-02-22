/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   cmr_holder.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   2019
*
* @brief  Classes to hold a message for publishing, receieving.
Contains additional checks for subscribers or if message was already sent.
*/
#pragma once
#include <ros/ros.h>
#include <string>
#include <mutex>
#include <memory>
namespace cmr_os {

  template <typename ROSMSG>
  class cmrHolder {
  public:
    cmrHolder() : is_new(false) {}

    void set_value(const ROSMSG& value) {
      mutex.lock();
      data = value;
      is_new = true;
      mutex.unlock();
    }

    bool get_value(ROSMSG& value) {
      bool was_new = false;

      mutex.lock();
      value = data;
      was_new = is_new;
      is_new = false;
      mutex.unlock();

      return was_new;
    }

    bool has_new_value() {
      return is_new;
    }

    ROSMSG get_value_unsynchronized() {
      return data;
    }


  private:
    ROSMSG data;
    bool is_new;
    std::mutex mutex;
  };

  template <typename ROSMSG>
    class cmrStateHolder {
    public:
      void init(const std::string& topic) {
        ros::NodeHandle nh;
        subscriber = nh.subscribe<ROSMSG>(topic, 1, &cmrStateHolder<ROSMSG>::set, this);
      }

      bool has_new_value() {
        return holder.has_new_value();
      }

      void set(ROSMSG value) {
        holder.set_value(value);
      }

      bool get(ROSMSG& value) {
        return holder.get_value(value);
      }

      ROSMSG get(){
        ROSMSG msg;
        holder.get_value(msg);
        return msg;
      }
    private:
      cmrHolder<ROSMSG> holder;
      ros::Subscriber subscriber;
    };


    template <typename ROSMSG>
    class cmrCommandHolder {
    public:
      void init(const std::string& topic) {
        ros::NodeHandle nh;
        publisher = nh.advertise<ROSMSG>(topic, 1);
      }

      void set(const ROSMSG& value) {
        holder.set_value(value);
      }

      ROSMSG get() {
        return holder.get_value_unsynchronized();
      }

      void publishIfNew() {
        static ROSMSG msg;
        if (publisher.getNumSubscribers() && holder.get_value(msg))
          publisher.publish(msg);
      }
    private:
      ros::Publisher publisher;
      cmrHolder<ROSMSG> holder;
    };
 }
