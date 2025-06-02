#pragma once

#include <utility>
#include <type_traits>
#include <concepts>
#include <Eigen/Core>
#include <Eigen/Geometry>

template <typename ScalarType, int Bits>
    requires(Bits > 0 && Bits <= 21)
static inline auto EncodeFloating(const ScalarType fp, const ScalarType maxAbs = 1.0)
{
    using UnsignedType = std::conditional_t<(Bits > 32), uint64_t, uint32_t>;
    UnsignedType fp_enc = static_cast<UnsignedType>((fp + maxAbs) / (2.0 * maxAbs) * ((UnsignedType(1) << Bits) - 1) + 0.5);
    return fp_enc;
}

template <typename ScalarType, int Bits>
    requires(Bits > 0 && Bits <= 21)
static inline auto DecodeFloating(const uint32_t fp_enc, const ScalarType maxAbs = 1.0)
{
    constexpr uint32_t maxInt = (uint32_t(1) << Bits) - 1;
    ScalarType fp = static_cast<ScalarType>(fp_enc & maxInt) / maxInt * (2.0 * maxAbs) - maxAbs;
    return fp;
}

template <typename ScalarType, int Bits>
    requires(Bits > 0 && Bits <= 21)
static inline auto Encode3D(const ScalarType x, const ScalarType y, const ScalarType z, const ScalarType maxAbs = 1.0)
{
    using UnsignedType = std::conditional_t<(Bits > 32), uint64_t, uint32_t>;
    auto x_enc = EncodeFloating<ScalarType, Bits>(x, maxAbs);
    auto y_enc = EncodeFloating<ScalarType, Bits>(y, maxAbs);
    auto z_enc = EncodeFloating<ScalarType, Bits>(z, maxAbs);
    UnsignedType fp_enc = z_enc | (y_enc << Bits) | (x_enc << (2 * Bits));
    return fp_enc;
}

template <typename ScalarType, int Bits>
    requires(Bits > 0 && Bits <= 21)
static inline auto Decode3D(const std::conditional_t<(Bits * 3 > 32), uint64_t, uint32_t> fp_enc, const ScalarType maxAbs = 1.0)
{
    ScalarType x = DecodeFloating<ScalarType, Bits>(fp_enc >> (2 * Bits), maxAbs);
    ScalarType y = DecodeFloating<ScalarType, Bits>(fp_enc >> (1 * Bits), maxAbs);
    ScalarType z = DecodeFloating<ScalarType, Bits>(fp_enc, maxAbs);
    return std::make_tuple(x, y, z);
}

template <typename ScalarType>
static inline auto EncodeQuaternions(const Eigen::Quaternion<ScalarType> &q1,
                                     const Eigen::Quaternion<ScalarType> &q2 = Eigen::Quaternion<ScalarType>::Identity(),
                                     const Eigen::Quaternion<ScalarType> &q3 = Eigen::Quaternion<ScalarType>::Identity())
{
    auto encodeQuat = [](const Eigen::Quaternion<ScalarType> &_q) -> uint64_t
    {
        using namespace std;
        uint8_t mark = 3;
        auto q = _q;
        q.normalize();
        ScalarType min = abs(q.w());
        ScalarType fp1, fp2, fp3;
        fp1 = q.x();
        fp2 = q.y();
        fp3 = q.z();
        if (abs(q.x()) < min)
        {
            min = abs(q.x());
            mark = 0;
            fp1 = q.y();
            fp2 = q.z();
            fp3 = q.w();
        }
        if (abs(q.y()) < min)
        {
            min = abs(q.y());
            mark = 1;
            fp1 = q.x();
            fp2 = q.z();
            fp3 = q.w();
        }
        if (abs(q.z()) < min)
        {
            min = abs(q.z());
            mark = 2;
            fp1 = q.x();
            fp2 = q.y();
            fp3 = q.w();
        }
        if (min > q.coeffs()(mark))
        {
            mark += 4;
        }

        auto fp1_enc = EncodeFloating<ScalarType, 13>(fp1);
        auto fp2_enc = EncodeFloating<ScalarType, 13>(fp2);
        auto fp3_enc = EncodeFloating<ScalarType, 13>(fp3);

        uint64_t result = uint64_t(0);

        result |= (uint64_t(mark) << 61);
        result |= (uint64_t(fp1_enc) << 48);
        result |= (uint64_t(fp2_enc) << 35);
        result |= (uint64_t(fp3_enc) << 22);
        return result;
    };

    auto q1_enc = encodeQuat(q1);
    auto q2_enc = encodeQuat(q2);
    auto q3_enc = encodeQuat(q3);

    uint64_t upper_bits = q1_enc | (q2_enc >> 42);
    uint64_t lower_bits = (q2_enc << 22) | (q3_enc >> 20);

    return std::make_pair(upper_bits, lower_bits);
}

template <typename ScalarType>
static inline auto DecodeQuaternions(const uint64_t upper_bits, const uint64_t lower_bits)
{
    auto decodeQuat = [](const uint64_t bin) -> Eigen::Quaternion<ScalarType>
    {
        uint8_t mark = (bin >> 61) & 0x7;
        ScalarType fp1, fp2, fp3;
        fp1 = DecodeFloating<ScalarType, 13>((bin >> 48) & 0x1FFF);
        fp2 = DecodeFloating<ScalarType, 13>((bin >> 35) & 0x1FFF);
        fp3 = DecodeFloating<ScalarType, 13>((bin >> 22) & 0x1FFF);
        ScalarType fp4_double = 1.0 - fp1 * fp1 - fp2 * fp2 - fp3 * fp3;
        ScalarType fp4 = fp4_double <= 0 ? 0 : sqrt(fp4_double);
        Eigen::Quaternion<ScalarType> result;
        if ((mark & 0x03) == 0)
        {
            result.x() = fp4;
            result.y() = fp1;
            result.z() = fp2;
            result.w() = fp3;
        }
        else if ((mark & 0x03) == 1)
        {
            result.x() = fp1;
            result.y() = fp4;
            result.z() = fp2;
            result.w() = fp3;
        }
        else if ((mark & 0x03) == 2)
        {
            result.x() = fp1;
            result.y() = fp2;
            result.z() = fp4;
            result.w() = fp3;
        }
        else if ((mark & 0x03) == 3)
        {
            result.x() = fp1;
            result.y() = fp2;
            result.z() = fp3;
            result.w() = fp4;
        }
        else if ((mark & 0x03) == 4)
        {
            result.x() = -fp4;
            result.y() = fp1;
            result.z() = fp2;
            result.w() = fp3;
        }
        else if ((mark & 0x03) == 5)
        {
            result.x() = fp1;
            result.y() = -fp4;
            result.z() = fp2;
            result.w() = fp3;
        }
        else if ((mark & 0x03) == 6)
        {
            result.x() = fp1;
            result.y() = fp2;
            result.z() = -fp4;
            result.w() = fp3;
        }
        else
        {
            result.x() = fp1;
            result.y() = fp2;
            result.z() = fp3;
            result.w() = -fp4;
        }
        return result;
    };

    auto q1 = decodeQuat(upper_bits);
    auto q2 = decodeQuat((upper_bits << 42) | (lower_bits >> 22));
    auto q3 = decodeQuat(lower_bits << 20);

    return std::make_tuple(q1, q2, q3);
}


template <typename ScalarType, int Bits>
    requires(Bits > 0 && Bits <= 21)
    [[nodiscard]]
static inline auto EncodeArmStatus(const ScalarType j1, const ScalarType j2, const ScalarType j3, const ScalarType j4, const ScalarType j5, const ScalarType j6, const ScalarType maxAbs = 1.0)
{
    using UnsignedType = std::conditional_t<(Bits * 6 > 32), std::uint64_t, std::uint32_t>;

    UnsignedType joint1 = static_cast<UnsignedType>(EncodeFloating<ScalarType, Bits>(j1, maxAbs));
    UnsignedType joint2 = static_cast<UnsignedType>(EncodeFloating<ScalarType, Bits>(j2, maxAbs));
    UnsignedType joint3 = static_cast<UnsignedType>(EncodeFloating<ScalarType, Bits>(j3, maxAbs));
    UnsignedType joint4 = static_cast<UnsignedType>(EncodeFloating<ScalarType, Bits>(j4, maxAbs));
    UnsignedType joint5 = static_cast<UnsignedType>(EncodeFloating<ScalarType, Bits>(j5, maxAbs));
    UnsignedType joint6 = static_cast<UnsignedType>(EncodeFloating<ScalarType, Bits>(j6, maxAbs));

    std::uint64_t fp_enc = joint1 | (joint2 << Bits) | (joint3 << (2 * Bits)) | 
        (joint4 << (3 * Bits)) | (joint5 << (4 * Bits)) | (joint6 << (5 * Bits));

    return fp_enc;
}

template <typename ScalarType, int Bits>
    requires(Bits > 0 && Bits <= 21)
    [[nodiscard]]
static inline auto DecodeArmStatus(const std::conditional_t<(Bits * 6 > 32), uint64_t, uint32_t> fp_enc, const ScalarType maxAbs = 1.0)
{
    ScalarType joint1 = DecodeFloating<ScalarType, Bits>(fp_enc >> (0 * Bits), maxAbs);
    ScalarType joint2 = DecodeFloating<ScalarType, Bits>(fp_enc >> (1 * Bits), maxAbs);
    ScalarType joint3 = DecodeFloating<ScalarType, Bits>(fp_enc >> (2 * Bits), maxAbs);
    ScalarType joint4 = DecodeFloating<ScalarType, Bits>(fp_enc >> (3 * Bits), maxAbs);
    ScalarType joint5 = DecodeFloating<ScalarType, Bits>(fp_enc >> (4 * Bits), maxAbs);
    ScalarType joint6 = DecodeFloating<ScalarType, Bits>(fp_enc >> (5 * Bits), maxAbs);

    return std::make_tuple(joint1, joint2, joint3, joint4, joint5, joint6);
}