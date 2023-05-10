/**
 * @file types.hpp
 * @author SeongChang Park (scsc1125@gmail.com)
 * @version 0.1
 * @date 2022-10-20 15:06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <queue>
#include <mutex>
#include <numeric>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#define   END    "\033[0m"
#define   RED    "\033[1;31m"
#define   GREEN  "\033[1;32m"
#define   BLUE   "\033[1;34m"