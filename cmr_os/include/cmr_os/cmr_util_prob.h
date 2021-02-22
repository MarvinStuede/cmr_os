/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   cmr_util_prob.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   2019
*
* @brief  Utility functions for probability distributions
*/

#pragma once
#include "ros/ros.h"
#include <math.h>
namespace cmr_os{
namespace util_prob{
/**
 * @brief Cumulative distribution function of the exponential distribution
 * @param rate
 * @param x
 * @return
 */
double expCumDist(double rate, double x);
/**
 * @brief Density function of the exponential distribution
 * @param rate
 * @param x
 * @return
 */
double expDist(double rate, double x);
};
};
