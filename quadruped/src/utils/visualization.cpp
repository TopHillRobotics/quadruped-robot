#include "utils/visualization.h"


StatisticAnalysis::StatisticAnalysis(float meanIn)
    :defaultMean(meanIn), var_(0.f), sigma_(0.f), num(0)
{
    mean_ = defaultMean;
    sum_ = defaultMean;
    data.clear();
}

void StatisticAnalysis::Update(float in)
{
    data.push_back(in);
    sum_ += in;
    num++;
}

float StatisticAnalysis::GetMean()
{
    mean_ = sum_ / num;
    return mean_;
}

float StatisticAnalysis::GetStandradVar()
{
    mean_ = sum_ / num;
    for (const float& item : data) {
        var_ += std::pow(item-mean_, 2);
    }
    var_ /= num;
    sigma_ = std::sqrt(var_);
    return sigma_;
}

void StatisticAnalysis::Reset(float meanIn) {
    defaultMean = meanIn;
    mean_ = defaultMean;
    sum_ = defaultMean;
    num = 0;
    var_ = 0;
    sigma_ = 0;
    data.clear();
}

void StatisticAnalysis::PrintStatistics()
{
    GetStandradVar();
    std::cout << "Mean = " << mean_
            << ",  Var = " << var_
            << ", Sigma = " << sigma_ << std::endl;
}

Visualization2D::Visualization2D()
{
    datax.clear();
    datay1.clear();
    datay2.clear();
    datay3.clear();
    datay4.clear();
    datay5.clear();
    labelNames = {"cur FL", "cur RR", "cur RL", "cur FR", "label"};
}

void Visualization2D::SetLabelNames(std::vector<std::string> labelNamesIn)
{
    if(labelNamesIn.size()<5) printf("ERROR IN Visualization2D::SetLabelNames\n");
    labelNames.clear();
    labelNames = labelNamesIn;
}

void Visualization2D::Show()
{
    int n = datax.size();
    std::cout << n << ", " << datay1.size() << ", " <<  datay2.size() << ", " <<
        datay3.size() << ", " << datay4.size() << ", " << datay5.size() <<std::endl;
    if (n>0){
        plt::figure();
        // plt::plot(datax,datay);
        std::map<std::string, std::string> map_;
        map_["cmap"] = "viridis";
        map_["alpha"] = "0.5";
        // map_["label"] = "ours";
        if (datay1.size() == n)
            plt::plot(datax,datay1, {{"label", labelNames[0]},{"linestyle", "-."}});
            // plt::scatter(datax,datay5, 1.0, map_);
        if (datay2.size() == n)
            plt::plot(datax,datay2, {{"label", labelNames[1]},{"linestyle", "--"}});
        if (datay3.size() == n)
            plt::plot(datax,datay3, {{"label", labelNames[2]},{"linestyle", "--"}});
        if (datay4.size() == n)
            plt::plot(datax,datay4, {{"label", labelNames[3]},{"linestyle", "--"}});
        // map_["label"] = "cmu";
            
        if (datay5.size() == n)
            plt::plot(datax,datay5, {{"label", labelNames[4]}, {"linestyle", ":"}});
            // plt::scatter(datay4,datay5, 1.0, map_);
        
        // std::vector<float> xticks = {2.5f, 5.0f, 7.5f, 10.f};
        // plt::xticks(xticks);
        plt::grid(true);
        plt::legend();
        plt::show();
        
    } else {
        throw std::domain_error("datax no data");
    }
    std::cout << "VISUALIZATION EXIT!" << std::endl;
    exit(0);
}

