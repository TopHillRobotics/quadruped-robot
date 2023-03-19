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

#ifndef QR_PRINT_H
#define QR_PRINT_H

#include <stdarg.h>
#include <stdio.h>
#include <iostream>
#include <string>

#include "qr_cpptypes.h"


enum class PrintColor {
    Default,
    Red,
    Green,
    Yellow,
    Blue,
    Magenta,
    Cyan
};

/**
 * @brief Printf, but with color.
 */
void printf_color(PrintColor color, const char *fmt, ...);

/**
 * @brief Fprintf, but with color (used to print color to STDERR).
 */
void fprintf_color(PrintColor color, FILE* stream, const char *fmt, ...);

/**
 * @brief Floating point value to string.
 */
template <typename T>
std::string pretty_string(T vv) {
    static int const buflen(32);
    static char buf[buflen];
    memset(buf, 0, sizeof(buf));
    snprintf(buf, buflen - 1, "% 6.6f  ", vv);
    std::string str(buf);
    return str;
}

template <typename T>
void pretty_print(DMat<T> const &mm, std::ostream &os, std::string const &title,
                  std::string const &prefix = "", bool vecmode = false,
                  bool nonl = false) {
    char const *nlornot("\n");
    if (nonl) {
        nlornot = "";
    }
    if (!title.empty()) {
        os << title << nlornot;
    }
    if ((mm.rows() <= 0) || (mm.cols() <= 0)) {
        os << prefix << " (empty)" << nlornot;
    } else {
        if (vecmode) {
            if (!prefix.empty()) os << prefix;
            for (int ir(0); ir < mm.rows(); ++ir) {
                os << pretty_string(mm.coeff(ir, 0));
            }
            os << nlornot;

        } else {
            for (int ir(0); ir < mm.rows(); ++ir) {
                if (!prefix.empty()) os << prefix;
                for (int ic(0); ic < mm.cols(); ++ic) {
                    os << pretty_string(mm.coeff(ir, ic));
                }
                os << nlornot;
            }
        }
    }
}

template <typename T>
void pretty_print(Quat<T> const &vv, std::ostream &os, std::string const &title,
                  std::string const &prefix = "", bool nonl = false) {
    pretty_print((DMat<T> const &)vv, os, title, prefix, true, nonl);
}

template <typename T>
void pretty_print(DVec<T> const &vv, std::ostream &os, std::string const &title,
                  std::string const &prefix = "", bool nonl = false) {
    pretty_print((DMat<T> const &)vv, os, title, prefix, true, nonl);
}

template <typename T>
void pretty_print(D3Mat<T> const &vv, std::ostream &os,
                  std::string const &title, std::string const &prefix = "",
                  bool nonl = false) {
    pretty_print((DMat<T> const &)vv, os, title, prefix, false, nonl);
}

template <typename T>
void pretty_print(Mat3<T> const &vv, std::ostream &os, std::string const &title,
                  std::string const &prefix = "", bool nonl = false) {
    pretty_print((DMat<T> const &)vv, os, title, prefix, false, nonl);
}

template <typename T>
void pretty_print(Mat6<T> const &vv, std::ostream &os, std::string const &title,
                  std::string const &prefix = "", bool nonl = false) {
    pretty_print((DMat<T> const &)vv, os, title, prefix, false, nonl);
}

template <typename T>
void pretty_print(SVec<T> const &vv, std::ostream &os, std::string const &title,
                  std::string const &prefix = "", bool nonl = false) {
    pretty_print((DMat<T> const &)vv, os, title, prefix, true, nonl);
}

template <typename T>
void pretty_print(Vec3<T> const &vv, std::ostream &os, std::string const &title,
                  std::string const &prefix = "", bool nonl = false) {
    pretty_print((DMat<T> const &)vv, os, title, prefix, true, nonl);
}

template <typename T>
void pretty_print(Vec2<T> const &vv, std::ostream &os, std::string const &title,
                  std::string const &prefix = "", bool nonl = false) {
    pretty_print((DMat<T> const &)vv, os, title, prefix, true, nonl);
}

template <typename T>
void pretty_print(const std::vector<T> &_vec, const char *title) {
    printf("%s: ", title);
    for (size_t i(0); i < _vec.size(); ++i) {
        printf("% 6.4f, \t", _vec[i]);
    }
    printf("\n");
}

template <typename T>
void pretty_print(const T *_vec, const char *title, size_t size) {
    printf("%s: ", title);
    for (size_t i(0); i < size; ++i) {
        printf("% 6.4f, \t", _vec[i]);
    }
    printf("\n");
}

template <class T>
void print_array(T* array, u16 rows, u16 cols) {
    for (u16 r = 0; r < rows; r++)
    {
        for (u16 c = 0; c < cols; c++)
            std::cout<<(fpt)array[c+r*cols]<<" ";
        printf("\n");
    }
}

template <class T>
void print_named_array(const char* name, T* array, u16 rows, u16 cols) {
    printf("%s:\n",name);
    print_array(array,rows,cols);
}

template <class T>
void pnv(const char* name, T v) {
    printf("%s: ",name);
    std::cout<<v<<std::endl;
}

#endif // QR_PRINT_H
