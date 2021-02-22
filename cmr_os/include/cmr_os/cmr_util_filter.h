/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   cmr_util_filter.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   2019
*
* @brief  Utility functions for filters
*/

#pragma once
#include "ros/ros.h"
namespace cmr_os{
namespace util_filter{
using namespace std;
//Simple moving average filter
template <typename T, typename Total, size_t N>
class SimpleMovingAverage
{
  public:
    void operator()(T sample)
    {
        if (num_samples_ < N)
        {
            samples_[num_samples_++] = sample;
            total_ += sample;
        }
        else
        {
            T& oldest = samples_[num_samples_++ % N];
            total_ += sample - oldest;
            oldest = sample;
        }
    }

    operator double() const { return total_ / min(num_samples_, N); }

  private:
    T samples_[N];
    size_t num_samples_{0};
    Total total_{0};
};
};
};
