#pragma once

#include "common_defines.h"

#include <tuple>
#include <dc/matrix.h>

#ifdef DC_SH4

#define mat_trans_nodiv_nomod(x, y, z, x2, y2, z2, w2) do { \
        register float __x __asm__("fr12") = (x); \
        register float __y __asm__("fr13") = (y); \
        register float __z __asm__("fr14") = (z); \
        register float __w __asm__("fr15") = 1.0f; \
        __asm__ __volatile__( "ftrv  xmtrx, fv12\n" \
                              : "=f" (__x), "=f" (__y), "=f" (__z), "=f" (__w) \
                              : "0" (__x), "1" (__y), "2" (__z), "3" (__w) ); \
        x2 = __x; y2 = __y; z2 = __z; w2 = __w; \
    } while(false)

#define mat_trans_w_nodiv_nomod(x, y, z, w) do { \
        register float __x __asm__("fr12") = (x); \
        register float __y __asm__("fr13") = (y); \
        register float __z __asm__("fr14") = (z); \
        register float __w __asm__("fr15") = 1.0f; \
        __asm__ __volatile__( "ftrv  xmtrx, fv12\n" \
                              : "=f" (__x), "=f" (__y), "=f" (__z), "=f" (__w) \
                              : "0" (__x), "1" (__y), "2" (__z), "3" (__w) ); \
        w = __w; \
    } while(false)

__always_inline float Fmac(float a, float b, float c) {
    asm volatile ("fmac fr0, %[floatb], %[floatc]\n"
        : [floatc] "+f" (c) : "w" (a), [floatb] "f" (b) : );
    return c;
}

#else

#define mat_trans_nodiv_nomod(x_, y_, z_, x2, y2, z2, w2) do { \
		vector_t tmp = { x_, y_, z_, 1.0f }; \
		mat_transform(&tmp, &tmp, 1, 0); \
		x2 = tmp.x; y2 = tmp.y; z2 = tmp.z; w2 = tmp.w; \
	} while(false)

#define mat_trans_w_nodiv_nomod(x_, y_, z_, w_) do { \
		vector_t tmp = { x_, y_, z_, 1.0f }; \
		mat_transform(&tmp, &tmp, 1, 0); \
		w_ = tmp.w; \
	} while(false)

__always_inline float Fmac(float a, float b, float c) { return a * b + c; }

#endif

__always_inline float Sin(float x) { return __builtin_sinf(x); }
__always_inline float Cos(float x) { return __builtin_cosf(x); }
__always_inline auto SinCos(float x) { return std::pair { Sin(x), Cos(x) }; }
__always_inline float Tan(float x) { return __builtin_tanf(x); }
__always_inline float Abs(float x) { return __builtin_fabsf(x); }
__always_inline float Sqrt(float x) { return __builtin_sqrtf(x); }
__always_inline float RecipSqrt(float x) { return 1.0f / __builtin_sqrtf(x); }
__always_inline float Asin(float x) { return __builtin_asinf(x); }
__always_inline float Acos(float x) { return __builtin_acosf(x); }
__always_inline float Atan(float x) { return __builtin_atanf(x); }
__always_inline float Atan2(float y, float x) { return __builtin_atan2f(y, x); }
__always_inline float RecipSqrt(float x, float y) { return x / __builtin_sqrtf(y); /*y = RecipSqrt(y); return x * y * y;*/ }
__always_inline float Pow(float x, float y) { return __builtin_powf(x, y); }
__always_inline float Floor(float x) { return __builtin_floorf(x); }
__always_inline float Ceil(float x) { return __builtin_ceilf(x); }
__always_inline float Invert(float x) { return (((x) < 0.0f)? -1.0f : 1.0f) * RecipSqrt((x) * (x)); } 
__always_inline float Div(float x, float y) { return x * Invert(y); }
__always_inline float Lerp(float a, float b, float t) { return Fmac(t, (b - a), a); }