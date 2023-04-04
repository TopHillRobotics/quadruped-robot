// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <cassert>
#include <iostream>

#include "utils/amd/include/amd.h"
#include "utils/qr_algebra.h"


/**
 * @brief Loop unroll for all inner loops, AVX2 only.
 */
static constexpr s64 UNROLL_MATVEC = 8;

#ifdef JCQP_USE_AVX2
__attribute__ ((unused))
static void spTriAVX(u32 n, const u32* p, const u32* i, const double* v, double* x)
{
    constexpr int UNROLL = 1;

    for (u32 c = 0; c < n; c++) {
        __m256d xV = _mm256_set1_pd(x[c]);

        s32 a = p[c];
        s32 end = p[c+1];
        s32 vEnd = end - 4 * UNROLL;
        const double* matPtr = v + a;
        const u32* iPtr = i + a;
        for (; a < vEnd; a += 4 * UNROLL) {
            for (int u = 0; u < UNROLL; u++) {
                __m256d mat = _mm256_loadu_pd(matPtr);
                __m256d lhs = _mm256_set_pd(x[iPtr[3]], x[iPtr[2]], x[iPtr[1]], x[iPtr[0]]);
                __m256d fma = _mm256_fnmadd_pd(xV, mat, lhs);

                x[iPtr[0]] = fma[0];
                x[iPtr[1]] = fma[1];
                x[iPtr[2]] = fma[2];
                x[iPtr[3]] = fma[3];

                matPtr += 4;
                iPtr += 4;
            }
        }

        for (; a < end; a++) {
            x[iPtr[0]] -= matPtr[0] * x[c];
            matPtr++;
            iPtr++;
        }
    }
}


__attribute__ ((unused))
static void spTriTAVX(u32 n, const u32* p, const u32* i, const double* v, double* x)
{
    constexpr int UNROLL = 1;
    for (s32 c = n - 1; c >= 0; c--) {
        s32 a = p[c];
        s32 end = p[c+1];
        s32 vEnd = end - 4 * UNROLL;
        const u32* iPtr = i + a;

        __m256d acc = _mm256_set1_pd(0.);

        for (; a < vEnd; a += 4 * UNROLL) {
            for (int u = 0; u < UNROLL; u++) {
                __m256d mat = _mm256_loadu_pd(v);
                __m256d rhs = _mm256_set_pd(x[iPtr[3]], x[iPtr[2]], x[iPtr[1]], x[iPtr[0]]);
                acc = _mm256_fmadd_pd(mat, rhs, acc);

                v += 4;
                iPtr += 4;
            }
        }

        x[c] -= acc[0];
        x[c] -= acc[1];
        x[c] -= acc[2];
        x[c] -= acc[3];

        for (;a < end; a++) {
            x[c] -= *(v++) * x[*(iPtr++)];
        }
    }
}


/**
 * @brief Backsubstitution solve L^{-T}x = b in place.
 * @param n: matrix size.
 * @param p: column pointers of sparse matrix.
 * @param i: row indices of sparse matrix.
 * @param v: values of sparse matrix.
 * @param x: input/output (x,b).
 */
__attribute__ ((unused))
static void spTriTAVX2(u32 n, const u32* p, const u32* i, const double* v, double* x)
{
    constexpr int UNROLL = 1;
    for (s32 c = n - 1; c >= 0; c--) {
        s32 colStart = p[c];
        s32 end = p[c+1];
        s32 vEnd = end - 4 * UNROLL;
        const u32* iPtr = i + colStart;

        const double* matPtr = v + colStart;
        __m256d acc = _mm256_set1_pd(0.);

        for (; colStart < vEnd; colStart += 4 * UNROLL) {
            for (int u = 0; u < UNROLL; u++) {
                __m256d mat = _mm256_loadu_pd(matPtr);
                __m256d rhs = _mm256_set_pd(x[iPtr[3]], x[iPtr[2]], x[iPtr[1]], x[iPtr[0]]);
                acc = _mm256_fmadd_pd(mat, rhs, acc);

                matPtr += 4;
                iPtr += 4;
            }
        }

        /* Horizontal sum and set x. */
        x[c] -= acc[0];
        x[c] -= acc[1];
        x[c] -= acc[2];
        x[c] -= acc[3];

        /* Whatever didn't fit in the vectors. */
        for (;colStart < end; colStart++) {
            x[c] -= *(matPtr++) * x[*(iPtr++)];
        }
    }
}
#endif
