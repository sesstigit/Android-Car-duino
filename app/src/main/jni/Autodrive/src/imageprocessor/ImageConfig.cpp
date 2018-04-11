/**
*    This file is part of Autodrive.
*
*    Autodrive is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    Autodrive is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with Autodrive.  If not, see <http://www.gnu.org/licenses/>.
**/
 
#include "ImageConfig.h"

using namespace Autodrive;

//Constructor
ImageConfig::ImageConfig() :
  normalize_lighting_(false),
  first_fragment_max_dist_(20),
  left_iteration_length_(6),
  right_iteration_length_(6),
  transform_line_removal_threshold_(12),
  use_left_line_(true),
  iterate_reduce_on_start_(-0.f),
  max_angle_diff_(1.0f),  //was 0.7 radians
  smoothening_(0),
  canny_thresh_(90) {
  }