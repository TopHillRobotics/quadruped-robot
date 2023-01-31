
namespace Quadruped {
    template<typename T>
    StateEstimatorContainer<T>::StateEstimatorContainer(Robot *quadrupedIn,
                                                GaitGenerator *gaitGeneratorIn,
                                                UserParameters *userParametersIn,
                                                std::string terrainConfigPath,
                                                std::string homeDir)
        : quadruped(quadrupedIn), gaitGenerator(gaitGeneratorIn), userParameters(userParametersIn)
    {
        groundEstimator = new GroundSurfaceEstimator(quadruped, homeDir + terrainConfigPath);
        
        contactDetection = new ContactDetection(quadruped,gaitGenerator, groundEstimator);
    
        robotEstimator = new RobotEstimator(quadruped, gaitGenerator, groundEstimator, userParametersIn);
        
        // _estimators.push_back(groundEsitmator);
        // _estimators.push_back(contactDetection);
        // _estimators.push_back(stateEstimator);
        std::cout << "init state estimator container!" << std::endl;    
    }
    

} // namespace Quadruped