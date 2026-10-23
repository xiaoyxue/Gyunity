#pragma once
#include "common/Core.h"
#include "common/Core.h"

GYT_NAMESPACE_BEGIN

inline uint32 ReverseBit32(uint32 n) {
	n = (n << 16) | (n >> 16);
	n = ((n & 0x00ff00ff) << 8) | ((n & 0xff00ff00) >> 8);
	n = ((n & 0x0f0f0f0f) << 4) | ((n & 0xf0f0f0f0) >> 4);
	n = ((n & 0x33333333) << 2) | ((n & 0xcccccccc) >> 2);
	n = ((n & 0x55555555) << 1) | ((n & 0xaaaaaaaa) >> 1);
	return n;
}

inline uint64 ReverseBit64(uint64 n) {
	uint64 n0 = ReverseBit32((uint32)n);
	uint64 n1 = ReverseBit32((uint32)(n >> 32));
	return (n0 << 32) | n1;
}

inline uint64 VanDerCorput(uint64 bits, const uint64 scramble)
{
    bits = (bits << 32) | (bits >> 32);
    bits = ((bits & 0x0000ffff0000ffffULL) << 16) |
        ((bits & 0xffff0000ffff0000ULL) >> 16);
    bits = ((bits & 0x00ff00ff00ff00ffULL) << 8) |
        ((bits & 0xff00ff00ff00ff00ULL) >> 8);
    bits = ((bits & 0x0f0f0f0f0f0f0f0fULL) << 4) |
        ((bits & 0xf0f0f0f0f0f0f0f0ULL) >> 4);
    bits = ((bits & 0x3333333333333333ULL) << 2) |
        ((bits & 0xccccccccccccccccULL) >> 2);
    bits = ((bits & 0x5555555555555555ULL) << 1) |
        ((bits & 0xaaaaaaaaaaaaaaaaULL) >> 1);
    return (scramble ^ bits) >> (64 - 52); // Account for 52 bits precision.
}

int IsPrime(int a) noexcept;

GYT_NAMESPACE_END