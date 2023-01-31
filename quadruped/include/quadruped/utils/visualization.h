/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: .
* Author: Zhu Yijie
* Create: 2022-03-30
* Notes: xx
* Modify: init the file. @ Zhu Yijie;
*/

#ifndef ASCEND_UTILS_VISUALIZATION_2D_H
#define ASCEND_UTILS_VISUALIZATION_2D_H
#include <ostream>
#include <vector>
#include <string>
#include <matplotlibcpp.h>
namespace plt = matplotlibcpp;

class StatisticAnalysis {
public:
    StatisticAnalysis(float mean=0.f);

    ~StatisticAnalysis() = default;

    void Update(float in);

    void Reset(float mean=0.f);

    float GetMean();

    float GetStandradVar();

    void PrintStatistics();

// private:
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
    std::vector<float> datay1,datay2,datay3,datay4,datay5;
    std::vector<std::string> labelNames;

    StatisticAnalysis sa[6];
    
};

#endif //ASCEND_UTILS_VISUALIZATION_2D_H
