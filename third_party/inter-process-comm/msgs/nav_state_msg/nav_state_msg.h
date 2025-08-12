#pragma once
#include <array>

struct nav_state_msg {
  using vec4 = std::array<float, 4>;
  using vec6 = std::array<float, 6>;
  // mask explanation
  // enable: 1
  // disable: 0
  // high:
  // low: pose request | pose answer | padding
  uint16_t mask;
  uint32_t cnt;
  uint64_t time;

  vec4 base_pos;
  vec4 base_quat;
  
  vec4 left_grip_one_pos;
  vec4 left_grip_one_quat;
  vec4 left_grip_two_pos;
  vec4 left_grip_two_quat;
  vec4 right_grip_one_pos;
  vec4 right_grip_one_quat;
  vec4 right_grip_two_pos;
  vec4 right_grip_two_quat;

  float left_force, right_force;
};