#include "controllers/state_dataflow.h"


UserParameters::UserParameters(std::string filePath)
{
    YAML::Node userConfig = YAML::LoadFile(filePath);
    stairsTime = userConfig["stairsTime"].as<float>();
    stairsVel = userConfig["stairsVel"].as<float>(); 
    controlFrequency = userConfig["controlFrequency"].as<unsigned int>();
    
    filterWindowSize = userConfig["filterWindowSize"].as<unsigned int>();
    
    accelerometerVariance = userConfig["accelerometerVariance"].as<float>();
    
    sensorVariance = userConfig["sensorVariance"].as<float>();
    
    initialVariance = userConfig["initialVariance"].as<float>();
    
    movingWindowFilterSize = userConfig["movingWindowFilterSize"].as<int>();
    
    desiredHeight = userConfig["desiredHeight"].as<float>();
    
    std::vector<float> desiredSpeed_ = userConfig["desiredSpeed"].as<std::vector<float>>();
    desiredSpeed = Eigen::MatrixXf::Map(&desiredSpeed_[0], 3, 1);
    
    desiredTwistingSpeed = userConfig["desiredTwistingSpeed"].as<float>();

    footClearance = userConfig["footClearance"].as<float>();
    std::vector<float> frictionCoeffs_ = userConfig["frictionCoeffs"].as<std::vector<float>>();
    frictionCoeffs = Eigen::MatrixXf::Map(&frictionCoeffs_[0], 4, 1);
    
    swingKp = userConfig["swingKp"].as<std::map<std::string, std::vector<float>>>();
    
    computeForceInWorldFrame = userConfig["computeForceInWorldFrame"].as<bool>();
    
    useWBC = userConfig["useWBC"].as<bool>();
    
    std::cout << "init UserParameters finish\n" ;
}
