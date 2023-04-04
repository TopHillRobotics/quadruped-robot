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

#ifndef QR_VISUALIZATION_H
#define QR_VISUALIZATION_H

#include <ostream>
#include <vector>
#include <string>


class StatisticAnalysis {

public:

    StatisticAnalysis(float mean=0.f);

    ~StatisticAnalysis() = default;

    void Update(float in);

    void Reset(float mean=0.f);

    float GetMean();

    float GetStandradVar();

    void PrintStatistics();

    float defaultMean;

    float mean_;

    float var_;

    float sigma_;

    float sum_;

    int num;

    std::vector<float> data;

};

class Visualization2D {

public:

    Visualization2D();

    ~Visualization2D() = default;

    void Show();

    void SetLabelNames(std::vector<std::string>);

    std::vector<float> datax;

    std::vector<float> datay1,datay2,datay3,datay4,datay5,datay6;

    std::vector<std::string> labelNames;

    StatisticAnalysis sa[6];

};

#endif // QR_VISUALIZATION_H
