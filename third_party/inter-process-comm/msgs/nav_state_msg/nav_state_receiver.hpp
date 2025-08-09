// Copyright 2025 Chuangye Liu <chuangyeliu0206@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "../common.hpp"
#include "../compress_utils.hpp"
#include "../CRC.h"
#include "nav_state_msg.h"

struct NavStateReceiver {
  using DataType                         = nav_state_msg;
  static constexpr uint8_t parser_type   = ParserType::Receiver;
  static constexpr uint16_t header       = 0xDCDC;
  static constexpr size_t length         = 34;
  static constexpr std::string_view name = "nav_state_receiver";

  static inline bool Process(const std::span<std::byte>& in, DataType& out) {
    constexpr float m_PI = 3.14159265358979323846f;
    // crc check
    {
      uint16_t crc = CRC::CalculateBits(in.data(), 32, CRC::CRC_16_KERMIT());
      uint16_t crc_in;
      memcpy(&crc_in, &in[32], sizeof(uint16_t));
      if (crc != crc_in) return false;
    }

    memcpy(&out.mask, &in[2], sizeof(uint16_t));
    memcpy(&out.cnt, &in[4], sizeof(uint32_t));
    memcpy(&out.time, &in[8], sizeof(uint64_t));

    uint32_t tmp_32bits;
    // base pos
    memcpy(out.base_pos.data(), &in[16], sizeof(float) * 3);
    // base quaternion
    memcpy(&tmp_32bits, &in[28], sizeof(uint32_t));
    DecodeQuaternion<float>(tmp_32bits, out.base_quat.data());
    return true;
  }
};