#include "ImageConfig.h"

//Constructor
ImageConfig::ImageConfig() :
  normalize_lighting_(true),
  first_fragment_max_dist_(30),
  left_iteration_length_(5),
  right_iteration_length_(6),
  transform_line_removal_threshold_(18),
  use_left_line_(true),
  iterate_reduce_on_start_(-2.f),
  max_angle_diff_(0.7f),
  smoothening_(0),
  // PID SETTINGS
  kp_(0.5),
  ki_(0.0),
  kd_(0.0) {
  }