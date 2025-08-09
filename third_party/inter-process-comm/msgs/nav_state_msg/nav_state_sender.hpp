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

struct NavStateSender {
  using DataType                         = nav_state_msg;
  static constexpr uint8_t parser_type   = ParserType::Sender;
  static constexpr uint16_t header       = 0xDCDC;
  static constexpr size_t length         = 34;
  static constexpr std::string_view name = "nav_state_sender";

  static inline void Process(const DataType& in, std::span<std::byte>& out) {
    constexpr float m_PI = 3.14159265358979323846f;
    uint32_t tmp_32bits;
    memcpy(&out[0], &header, sizeof(uint16_t));
    memcpy(&out[2], &in.mask, sizeof(uint16_t));

    memcpy(&out[4], &in.cnt, sizeof(uint32_t));
    memcpy(&out[8], &in.time, sizeof(uint64_t));
    // base pos
    memcpy(&out[16], in.base_pos.data(), sizeof(float) * 3);
    tmp_32bits = EncodeQuaternion<float>(in.base_quat.data());
    memcpy(&out[28], &tmp_32bits, sizeof(uint32_t));
    // CRC
    uint16_t crc = CRC::CalculateBits(out.data(), 32, CRC::CRC_16_KERMIT());
    memcpy(&out[32], &crc, sizeof(uint16_t));
    // total 34 bytes
    return;
  }
};