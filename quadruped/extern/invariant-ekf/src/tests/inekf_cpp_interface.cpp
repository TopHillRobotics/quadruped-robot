#include "inekf_cpp_interface.h"

using namespace std;
using namespace UNITREE_LEGGED_SDK;

#define DT_MIN 1e-6
#define DT_MAX 1

double stod98(const std::string &s) {
    return atof(s.c_str());
}

int stoi98(const std::string &s) {
    return atoi(s.c_str());
}

namespace inekf {

void INEKFInterface::printState()
{
    cout << "Robot's state is: \n";
    cout << filter.getState() << endl;
}

const Eigen::MatrixXd INEKFInterface::getRobotStateX()
{
    return filter.getState().getX();
}

const Eigen::VectorXd INEKFInterface::getRobotStateTheta(){
    return filter.getState().getTheta();
}
const Eigen::MatrixXd INEKFInterface::getRobotStateP(){
    return filter.getState().getP();
}

Vec3<double> INEKFInterface::getPosition()
{
    return filter.getState().getPosition();
}

Vec3<double> INEKFInterface::getRotation()
{
    Vec3<double> rpy = robotics::math::rotationMatrixToRPY(filter.getState().getRotation().transpose());
    return rpy;
}


void INEKFInterface::Initialize(Eigen::Matrix3d _R0, Eigen::Vector3d _v0, 
                                Eigen::Vector3d _bg0, Eigen::Vector3d _ba0){
    // R0 << 1, 0, 0, // initial orientation
    //       0, -1, 0, // IMU frame is rotated 90deg about the x-axis
    //       0, 0, -1;
    R0 << 1, 0, 0, // initial orientation
          0, 1, 0, 
          0, 0, 1;
    v0 << 0.0, 0.0, 0.0; // initial velocity
    p0 << 0.0, 0.0, 0.0; // initial position
    // bg0 << 0,0,0; // initial gyroscope bias
    // ba0 << 0,0,0; // initial accelerometer bias
    // R0 = _R0;
    // v0 = _v0;
    bg0 = _bg0;
    ba0 = _ba0;
    initial_state.setRotation(R0);
    initial_state.setVelocity(v0);
    initial_state.setPosition(p0);
    initial_state.setGyroscopeBias(bg0);
    initial_state.setAccelerometerBias(ba0);

    // // Initialize state covariance
    // initial_state.setRotationCovariance(0.03*Eigen::Matrix3d::Identity());
    // initial_state.setVelocityCovariance(0.01*Eigen::Matrix3d::Identity());
    // initial_state.setPositionCovariance(0.00001*Eigen::Matrix3d::Identity());
    // initial_state.setGyroscopeBiasCovariance(0.0001*Eigen::Matrix3d::Identity());
    // initial_state.setAccelerometerBiasCovariance(0.0025*Eigen::Matrix3d::Identity());
    
    cout << "bg0 = "<<bg0<<endl;
    cout << "ba0 = "<<ba0<<endl; 
    // Initialize state covariance
    noise_params.setGyroscopeNoise(0.01);
    noise_params.setAccelerometerNoise(0.01);
    noise_params.setGyroscopeBiasNoise(0.0001);
    noise_params.setAccelerometerBiasNoise(0.0025);
    noise_params.setContactNoise(0.1);

    // Initialize invariant extended Kalman filter
    filter.setState(initial_state);
    filter.setNoiseParams(noise_params);
    cout << "Noise parameters are initialized to: \n";
    cout << filter.getNoiseParams() << endl;
    cout << "Robot's state is initialized to: \n";
    cout << filter.getState() << endl;
    cout << robotics::math::rotationMatrixToRPY(filter.getState().getRotation().transpose())<<endl; 

    
    /*
    // Landmark 1
    // id = 1;
    // p_wl << 0,-1,0;
    // prior_landmarks.insert(pair<int,Eigen::Vector3d> (id, p_wl)); 

    // Landmark 2
    // id = 2;
    // p_wl << 1,1,-0.5;
    // prior_landmarks.insert(pair<int,Eigen::Vector3d> (id, p_wl)); 

    // Landmark 3
    // id = 3;
    // p_wl << 2,-1,0.5;
    // prior_landmarks.insert(pair<int,Eigen::Vector3d> (id, p_wl)); 

    // Store landmarks for localization
    // filter.setPriorLandmarks(prior_landmarks);
    */
}

void INEKFInterface::ReceiveMotorAngleData(double T, std::array<float, 12>& motor_angles)
{
    t = T;
}

void INEKFInterface::ReceiveImuDataInSim(double T, const IMU& imu)
{
    // # std::array<float, 4> quaternion;         // quaternion, normalized, (w,x,y,z)
    // # std::array<float, 3> gyroscope;          // angular velocity （unit: rad/s)
    // # std::array<float, 3> accelerometer;      // m/(s2)
    // # std::array<float, 3> rpy;                // euler angle（unit: rad)
    // # int8_t temperature;
    t = T;
   
    // std::array<float, 3> gyroscope = imu.gyroscope;
    // std::array<float, 3> accelerometer = imu.accelerometer;
    
    imu_measurement << imu.gyroscope[0], 
                       imu.gyroscope[1], 
                       imu.gyroscope[2],
                       imu.accelerometer[0],
                       imu.accelerometer[1],
                       imu.accelerometer[2];
    cout<<imu_measurement<<endl;
}

void INEKFInterface::ReceiveKinematicsMeasurement(const KinematicsMeasurement &kinematicsMeasurement)
{
    measured_kinematics.clear();
    q.clear();
    p.clear();
    std::vector<Eigen::Matrix3d> Jacobians;
    // cout<<"Kinematics:"<<endl;
    if (kinematicsMeasurement.q.size()==4) {
        auto& q_ = kinematicsMeasurement.q;

        for (auto& q_i : q_) {
            Eigen::Quaterniond qq(q_i[0], q_i[1],  q_i[2],  q_i[3]);

            // cout << qq.coeffs() << endl;
            q.push_back(qq);
        }
        // cout << "==========" <<endl;
        // std::cout << q[0] << std::endl;
    } else {
        throw exception();
    }
    
    if (kinematicsMeasurement.p.size()==4) {
        auto &p_ = kinematicsMeasurement.p;
        for (int i=0; i<4; ++i) {
            p.push_back(p_[i].cast<double>());
        }
        
    } else {
        throw exception();
    }

    //  analysis jacobian for each leg
    if (kinematicsMeasurement.J.size()==4) {
        auto& J_ = kinematicsMeasurement.J;
        for (auto& jj :J_){
            Jacobians.push_back(jj.cast<double>());
        }
    } else {
        throw exception();
    }
    
    const Eigen::Matrix3d Cov_e = Eigen::Matrix3d::Identity()*0.0174533*0.0174533; // 1 deg std dev 
    const Eigen::Matrix<double,3,3> prior_kinematics_cov = 0.05*0.05 * Eigen::Matrix<double,3,3>::Identity(); // 5 cm std Adds to FK covariance
    for (int i=0; i<4; ++i) {
        q[i].normalize();  // error
        pose.block<3,3>(0,0) = q[i].toRotationMatrix();
        pose.block<3,1>(0,3) = p[i];
        covariance = Eigen::Matrix<double,6,6>::Identity();
        Eigen::Matrix<double, 3, 3> J = Jacobians[i];
        // J.block<3,3>(3,0) = Jacobians[i];
        covariance.block<3,3>(3,3) =  J*Cov_e*J.transpose()+ prior_kinematics_cov; // --> 6*6 matrix
        measured_kinematics.push_back(Kinematics(i, pose, covariance));
    }
}

void INEKFInterface::ReceiveImuData(double dT, const IMU& imu)
{
    // cout << "Received IMU Data, propagating state\n";
    // assert((measurement.size()-2) == 6);
    dt = dT;
    // Angular Velocity (w), Linear Acceleration (a)
    imu_measurement << (double)imu.gyroscope[0], 
                       (double)imu.gyroscope[1], 
                       (double)imu.gyroscope[2],
                       (double)imu.accelerometer[0],
                       (double)imu.accelerometer[1],
                       (double)imu.accelerometer[2];
    // std::cout << "==\n" <<imu_measurement <<std::endl;
    IMUquat << (double)imu.quaternion[0], (double)imu.quaternion[1], (double)imu.quaternion[2],(double)imu.quaternion[3];
    // std::cout <<IMUrpy <<std::endl;
}

void INEKFInterface::UpdateContact(Eigen::Matrix<bool, 4, 1> is_contacts)
{
    // cout << "Received CONTACT Data, setting filter's contact state\n";
    vector<pair<int,bool> > contacts;
    int id;
    bool indicator;
    // Read in contact data
    for (int i=0; i<4; ++i) {
        id = i;
        indicator = is_contacts[i];
        contacts.push_back(pair<int,bool> (id, indicator));
    }       
    // Set filter's contact state
    filter.setContacts(contacts);
}

RobotState INEKFInterface::Update_1(Vec3<double> vInBaseFrame)
{
    // Propagate
    // Propagate using IMU data
    Eigen::Matrix3d oritationMatrix = robotics::math::quaternionToRotationMatrix(IMUquat);
    
    // double dt = t - t_prev;
    if (dt > DT_MIN && dt < DT_MAX) {
        // filter.setVelocity(oritationMatrix*vInBaseFrame);   // zhuyijie
        // filter.Propagate(imu_measurement_prev, dt);
        filter.Propagate(imu_measurement, dt);
    }
    // if (cycle%5==0)
    //     filter.setOrientation(oritationMatrix);
    // cycle++;
    // Store previous timestamp
    // t_prev = t;
    imu_measurement_prev = imu_measurement;
    // Print Propagated state
    const RobotState& state = filter.getState();
    // cout << state << endl;
    return state;
}

RobotState INEKFInterface::Update_2()
{
    // Correct
    filter.CorrectKinematics(measured_kinematics);
    // Print final state
    const RobotState& state = filter.getState();
    // cout << state << endl;
    return state;
}
} // inekf



// int main() 
// {
//     inekf::INEKFInterface inekf_;
//     return 0;
// }