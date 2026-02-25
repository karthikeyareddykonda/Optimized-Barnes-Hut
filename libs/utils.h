#pragma once
#include "baseline.h"
#include <vector>
#include <cfloat>
#include <fstream>
#include <iomanip>

#include <immintrin.h>

inline __m256d mask_from_4bit_opt(int mask4)
{
    // Can be handled better with 64 bit masks
    // Can be handled even better with full fledged double mask

    // Step 1: Expand 4-bit mask into 4 64-bit values using AVX2
    // Shift and mask each bit directly
    __m256i mask_i64 = _mm256_set_epi64x(
        -(int64_t)((mask4 >> 3) & 1), // Casting involved twice. to int64 and to double !.. can be expensive
        -(int64_t)((mask4 >> 2) & 1),
        -(int64_t)((mask4 >> 1) & 1),
        -(int64_t)((mask4 >> 0) & 1));

    // Step 2: Bitwise reinterpret as double mask
    return _mm256_castsi256_pd(mask_i64);
}

inline __m256d get_1_0_multiplier(int mask)
{
    // 1. Map bits to 32-bit integer lanes: [bit3, bit2, bit1, bit0]
    // We use a small bit-trick to isolate each bit into its own 32-bit lane
    __m128i v_mask = _mm_set1_epi32(mask);
    __m128i v_shift = _mm_set_epi32(3, 2, 1, 0);

    // Shift the bits so the target bit is at the LSB position of each lane
    __m128i v_isolated = _mm_srlv_epi32(v_mask, v_shift);

    // Mask out everything but the LSB: now lanes are exactly 1 or 0
    __m128i v_ints = _mm_and_si128(v_isolated, _mm_set1_epi32(1));

    // 2. Convert the 4 integers [1, 0, 1, 0] to 4 doubles [1.0, 0.0, 1.0, 0.0]
    return _mm256_cvtepi32_pd(v_ints);
}

inline std::pair<double, double> find_min_max(const std::vector<Body> &Bodies)
{
    double res_min = DBL_MAX, res_max = DBL_MIN;
    for (const Body &b : Bodies)
    {
        Vector3D v = b.pos;
        res_min = std::min({v.x, v.y, v.z, res_min});
        res_max = std::max({v.x, v.y, v.z, res_max});
    }
    return std::make_pair(res_min, res_max);
}

inline void write_to_file(const std::vector<Body> &Bodies, std::string filename)
{

    std::ofstream file(filename);
    file << Bodies.size() << "\n";
    int N = Bodies.size();
    std::vector<Body> Bodies_orig(N);

    for (const auto &b : Bodies)
    {
        Bodies_orig[b.index] = b; // deep copy
    }
    for (const auto &b : Bodies_orig)
    {
        file << std::fixed << std::setprecision(10) << b.pos.x << "," << b.pos.y << "," << b.pos.z << "," << b.vel.x << "," << b.vel.y << "," << b.vel.z << "," << b.mass << "\n";
    }

    file << std::flush;
}
