#include "RobotM1.h"

using namespace Eigen;

RobotM1::RobotM1(std::string robotName) : Robot(), calibrated(false), maxEndEffVel(2), maxEndEffForce(60), robotName_(robotName) {
    // Conversion factors between degrees and radians
    d2r = M_PI / 180.;
    r2d = 180. / M_PI;

    //Define the robot structure: each joint with limits and drive - TMH
    // JOINT 0 - the only joint in the case of M1
    max_speed(0) = 360; // {radians}
    tau_max(0) = 1.9 * 23;  // {Nm}
    LinkLengths(0) = 0.1;   // Link lengths used for kinematic models (in m)
    LinkMasses(0) = 0.5;    // Link masses used for gravity compensation (in kg)
    CoGLengths(0) = 0.08;    // Length along link(s) to the Center og Gravity
    Zero2GravityAngle(0) = 0;    // Angle from q=0 to the direction of gravitational force
    // g is the gravitational constant affecting that link, if its motion isn't subject to gravity set to 0
    g(0) = 9.81;  //Gravitational constant: remember to change it if using the robot on the Moon or another planet

    // Calibration configuration: posture in which the robot is when using the calibration procedure
    qCalibration(0) = 0 * d2r;
    tau_motor(0) = 0;
    q_filt(0) = 0;
    dq_filt(0) = 0;
    tau_s_filt(0) = 0;
    q_filt_pre(0) = 0;
    dq_filt_pre(0) = 0;
    tau_s_filt_pre(0) = 0;

    // Initializing the yaml parameters to zero
    m1Params.configFlag = true;
    m1Params.c0 = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.c1 = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.c2 = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.i_sin = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.i_cos = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.t_bias = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.force_sensor_scale_factor = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.force_sensor_offset = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.ff_ratio = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.kp = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.ki = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.kd = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.vel_thresh = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.tau_thresh = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.lowpass_cutoff_freq = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.motor_torque_cutoff_freq = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.tick_max = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.spk = Eigen::VectorXd::Zero(M1_NUM_JOINTS);
    m1Params.tracking_offset = Eigen::VectorXd::Zero(M1_NUM_JOINTS);

    initializeRobotParams(robotName_);

    // Set up the motor profile
    posControlMotorProfile.profileVelocity = 600.*512*10000/1875;
    posControlMotorProfile.profileAcceleration = 500.*65535*10000/4000000;
    posControlMotorProfile.profileDeceleration = 500.*65535*10000/4000000;

    initialiseJoints();
    initialiseInputs();

    mode = 0;

    status = R_SUCCESS;
}

RobotM1::~RobotM1() {
    spdlog::debug(" Delete RobotM1 object begins");

    for (auto p : joints) {
        spdlog::debug(" Delete Joint ID: {}.", p->getId());
        delete p;
    }
    joints.clear();
    inputs.clear();
    delete keyboard;
    spdlog::debug("RobotM1 deleted");
}

bool RobotM1::initialiseJoints() {
    joints.push_back(new JointM1(0, -100, 100, 1, -max_speed(0), max_speed(0), -tau_max(0), tau_max(0), new KincoDrive(1), "q1"));
    return true;
}

bool RobotM1::initialiseNetwork() {
    bool status;
    for (auto joint : joints) {
        status = joint->initNetwork();
        if (!status)
            return false;
    }
    //Give time to drives PDO initialisation
    //TODO: Parameterize the number of PDOs for situations like the one below
    //
    // spdlog::debug("...");
    // for (uint i = 0; i < 5; i++) {
    //     spdlog::debug(".");
    //     usleep(10000);
    // }
    return true;
}

bool RobotM1::initialiseInputs() {
    inputs.push_back(keyboard = new Keyboard());
    inputs.push_back(m1ForceSensor = new FourierForceSensor(17,
                                                            m1Params.force_sensor_scale_factor[0],
                                                            1,
                                                            m1Params.force_sensor_offset[0]));
    return true;
}

bool RobotM1::initializeRobotParams(std::string robotName) {

    // need to use address of base directory because when run with ROS, working directory is ~/.ros
    std::string baseDirectory = XSTR(BASE_DIRECTORY);
    std::string relativeFilePath = "/CANOpenRobotController/config/m1_params.yaml";

    YAML::Node params = YAML::LoadFile(baseDirectory + relativeFilePath);

    // if the robotName does not match with the name in m1_params.yaml
    if(!params[robotName]){
        spdlog::error("Parameters of {} couldn't be found in {} !", robotName, baseDirectory + relativeFilePath);
        spdlog::error("All parameters are zero !");
        return false;
    }

    if(params[robotName]["c0"].size() != M1_NUM_JOINTS || params[robotName]["c1"].size() != M1_NUM_JOINTS ||
       params[robotName]["c2"].size() != M1_NUM_JOINTS || params[robotName]["i_sin"].size() != M1_NUM_JOINTS ||
       params[robotName]["i_cos"].size() != M1_NUM_JOINTS || params[robotName]["t_bias"].size() != M1_NUM_JOINTS ||
       params[robotName]["force_sensor_scale_factor"].size() != M1_NUM_JOINTS ||
       params[robotName]["force_sensor_offset"].size() != M1_NUM_JOINTS ||
       params[robotName]["ff_ratio"].size() != M1_NUM_JOINTS || params[robotName]["kp"].size() != M1_NUM_JOINTS ||
       params[robotName]["ki"].size() != M1_NUM_JOINTS || params[robotName]["kd"].size() != M1_NUM_JOINTS ||
       params[robotName]["vel_thresh"].size() != M1_NUM_JOINTS || params[robotName]["tau_thresh"].size() != M1_NUM_JOINTS ||
       params[robotName]["lowpass_cutoff_freq"].size() != M1_NUM_JOINTS ||
       params[robotName]["motor_torque_cutoff_freq"].size() != M1_NUM_JOINTS ||
       params[robotName]["tick_max"].size() != M1_NUM_JOINTS || params[robotName]["spk"].size() != M1_NUM_JOINTS ||
       params[robotName]["tracking_offset"].size() != M1_NUM_JOINTS) {

        spdlog::error("Parameter sizes are not consistent");
        spdlog::error("All parameters are zero !");

        return false;
    }

    // getting the parameters from the yaml file
    for(int i = 0; i<M1_NUM_JOINTS; i++){
        m1Params.configFlag = params["config_flag"].as<bool>();
        m1Params.c0[i] = params[robotName]["c0"][i].as<double>();
        m1Params.c1[i] = params[robotName]["c1"][i].as<double>();
        m1Params.c2[i] = params[robotName]["c2"][i].as<double>();
        m1Params.i_sin[i] = params[robotName]["i_sin"][i].as<double>();
        m1Params.i_cos[i] = params[robotName]["i_cos"][i].as<double>();
        m1Params.t_bias[i] = params[robotName]["t_bias"][i].as<double>();
        m1Params.force_sensor_scale_factor[i] = params[robotName]["force_sensor_scale_factor"][i].as<double>();
        m1Params.force_sensor_offset[i] = params[robotName]["force_sensor_offset"][i].as<double>();
        m1Params.ff_ratio[i] = params[robotName]["ff_ratio"][i].as<double>();
        m1Params.kp[i] = params[robotName]["kp"][i].as<double>();
        m1Params.ki[i] = params[robotName]["ki"][i].as<double>();
        m1Params.kd[i] = params[robotName]["kd"][i].as<double>();
        m1Params.vel_thresh[i] = params[robotName]["vel_thresh"][i].as<double>();
        m1Params.tau_thresh[i] = params[robotName]["tau_thresh"][i].as<double>();
        m1Params.lowpass_cutoff_freq[i] = params[robotName]["lowpass_cutoff_freq"][i].as<double>();
        m1Params.motor_torque_cutoff_freq[i] = params[robotName]["motor_torque_cutoff_freq"][i].as<double>();
        m1Params.tick_max[i] = params[robotName]["tick_max"][i].as<double>();
        m1Params.spk[i] = params[robotName]["spk"][i].as<double>();
        m1Params.tracking_offset[i] = params[robotName]["tracking_offset"][i].as<double>();
    }

    // Set static parameter values
    i_sin_ = m1Params.i_sin[0];
    i_cos_ = m1Params.i_cos[0];
    t_bias_ = m1Params.t_bias[0];
    f_s_ = m1Params.c0[0];
    f_d_ = m1Params.c1[0];
    c2_ = m1Params.c2[0];
    q_offset_ = m1Params.tracking_offset[0]*d2r; // radians
    if (!m1Params.configFlag) {
        velThresh_ = m1Params.vel_thresh[0] * d2r;
        torqueThresh_ = m1Params.tau_thresh[0];
        motorTorqueCutOff_ = m1Params.motor_torque_cutoff_freq[0];
    }
    tau_offset_ = 0; // Nm
    tau_df_ = 1; // Nm
    tau_pf_ = 1; // Nm
    q_df_ = 0; // radians
    q_pf_ = 0; // radians
    stim_df_ = 0;
    stim_pf_ = 0;
    stim_calib_ = false;
    return true;
}

RobotParameters RobotM1::sendRobotParams() {
    return m1Params;
}

bool RobotM1::stop() {
    std::cout << "Stopping M1 robot..." << std::endl;
    for (auto p : joints) {
        ((JointM1 *)p)->disable();
    }
    return true;
}

void RobotM1::applyCalibration() {
    for (uint i = 0; i < joints.size(); i++) {
        ((JointM1 *)joints[i])->setPositionOffset(qCalibration(i));
    }
    calibrated = true;
}

bool RobotM1::calibrateForceSensors() {
    //if(m1ForceSensor->calibrate()){
    //    spdlog::debug("[RobotM1::calibrateForceSensors]: Zeroing of force sensors are successfully completed.");
    //    return true;
    //} else{
    //    spdlog::debug("[RobotM1::calibrateForceSensors]: Zeroing failed.");
    //    return false;
    //}

    if(m1ForceSensor->sendInternalCalibrateSDOMessage()){
        spdlog::info("[RobotM1::calibrateForceSensors]: Force sensor zeroing completed. Sensor value is {}", m1ForceSensor->getSensorValue());
        return true;
    } else {
        spdlog::info("[RobotM1::calibrateForceSensors]: Zeroing failed.");
        return false;
    }
}

void RobotM1::updateRobot() {
    Robot::updateRobot();   // Trigger RT data update at the joint level
    // Gather joint data at the Robot level
    //TODO: we should probably do this with all PDO data, if we are setting it up
    // to be delivered in real-time we should make it available and use it or stop
    // sending it in real-time to conserve bandwidth. It would also be good to break
    // down the status word into a vector of booleans and have descriptive indices to
    // be able to clearly access the bits in a meaningful and readable way. -TMH

    for(uint i = 0; i < nJoints; i++) {
        q(i) = ((JointM1 *)joints[i])->getPosition();
        dq(i) = ((JointM1 *)joints[i])->getVelocity();
        tau(i) = ((JointM1 *)joints[i])->getTorque();
        tau_s(i) = m1ForceSensor[i].getForce();
//        updateEstimatedAcceleration(); // based on velocity measurements

        //std::cout << std::setprecision(4) << tau_s(i) << std::endl;

        // compensate inertia for torque sensor measurement (comment this line when zeroing force sensor)
        tau_s(i) =  tau_s(i) + i_sin_*sin(q(i)+t_bias_) + i_cos_*cos(q(i)+t_bias_);
    }
    if (safetyCheck() != SUCCESS) {
        status = R_OUTSIDE_LIMITS;
        stop();
    }
}

bool RobotM1::setDigitalOut(int digital_out) {
    ((JointM1 *)joints[0])->setDigitalOut(digital_out);
    return true;
}

int RobotM1::getDigitalIn() {
    return ((JointM1 *)joints[0])->getDigitalIn();
}

setMovementReturnCode_t RobotM1::safetyCheck() {
    //End-effector safeties if calibrated
    for (uint i = 0; i < nJoints; i++) {    // Error found, YW
        if (((JointM1 *)joints[i])->safetyCheck() != SUCCESS) {
            std::cout /*cerr is banned*/ << "M1: Joint " << i << " safety triggered!" << std::endl;
            return OUTSIDE_LIMITS;
        }
    }
    return SUCCESS;
}

void RobotM1::printStatus() {
    std::cout << std::setprecision(3) << std::fixed;
//    std::cout << "X=[ " << getEndEffPos().transpose() << " ]\t";
//    std::cout << "dX=[ " << getEndEffVel().transpose() << " ]\t";
//    std::cout << "F=[ " << getEndEffFor().transpose() << " ]\t";
    std::cout << std::endl;
}

void RobotM1::printJointStatus() {
    std::cout << std::setprecision(3) << std::fixed;
    std::cout << "q=[ " << getJointPos().transpose() << " ]\t";
    std::cout << "dq=[ " << getJointVel().transpose() << " ]\t";
    std::cout << "tau=[ " << getJointTor().transpose() << " ]\t";
    std::cout << "tau_s=[ " << getJointTor_s().transpose() << " ]\t";
    std::cout << "{";
    for (auto joint : joints)
        std::cout << "0x" << std::hex << ((JointM1 *)joint)->getDriveStatus() << "; ";
    std::cout << "}" << std::endl;
}

bool RobotM1::initMonitoring() {
    spdlog::debug("Initialising monitoring all joints.");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM1 *)p)->setMode(CM_POSITION_CONTROL, posControlMotorProfile) != CM_POSITION_CONTROL) {
            // Something bad happened if we are here
            spdlog::debug("Something bad happened.");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
//        ((JointM1 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM1 *)p)->disable();
    }
    return returnValue;
}

bool RobotM1::initPositionControl() {
    spdlog::debug("Initialising Position Control on all joints.");
    bool returnValue = true;

    for (auto p : joints) {
        if (((JointM1 *)p)->setMode(CM_POSITION_CONTROL, posControlMotorProfile) != CM_POSITION_CONTROL) {
            // Something bad happened if we are here
            spdlog::debug("Something bad happened.");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
//        ((JointM1 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
   usleep(2000);
   for (auto p : joints) {
       ((JointM1 *)p)->enable();
   }
    mode = 1;
    return returnValue;
}

bool RobotM1::initVelocityControl() {
    spdlog::debug("Initialising Velocity Control on all joints.");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM1 *)p)->setMode(CM_VELOCITY_CONTROL, posControlMotorProfile) != CM_VELOCITY_CONTROL) {
            // Something bad happened if we are here
            spdlog::debug("Something bad happened.");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointM1 *)p)->readyToSwitchOn();
    }
    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM1 *)p)->enable();
    }
    mode = 2;
    return returnValue;
}

bool RobotM1::initTorqueControl() {
    spdlog::debug("Initialising Torque Control on all joints.");
    bool returnValue = true;
    for (auto p : joints) {
        if (((JointM1 *)p)->setMode(CM_TORQUE_CONTROL) != CM_TORQUE_CONTROL) {
            // Something bad happened if we are here
            spdlog::debug("Something bad happened.");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        ((JointM1 *)p)->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        ((JointM1 *)p)->enable();
    }
    mode = 3;
    return returnValue;
}

setMovementReturnCode_t RobotM1::applyPosition(JointVec positions) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    for (auto p : joints) {
        setMovementReturnCode_t setPosCode = ((JointM1 *)p)->setPosition(positions(i));
        if (setPosCode == INCORRECT_MODE) {
            std::cout << "Joint " << p->getId() << ": is not in Position Control " << std::endl;
            returnValue = INCORRECT_MODE;
        } else if (setPosCode != SUCCESS) {
            // Something bad happened
            std::cout << "Joint " << p->getId() << ": " << std::endl;
            ((JointM1 *)p)->errorMessage(setPosCode);
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}

setMovementReturnCode_t RobotM1::applyVelocity(JointVec velocities) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    for (auto p : joints) {
//        std::cout << "Joint velocity 2: " << velocities(0) << std::endl;
        setMovementReturnCode_t setVelCode = ((JointM1 *)p)->setVelocity(velocities(i));
        if (setVelCode == INCORRECT_MODE) {
            std::cout << "Joint " << p->getId() << ": is not in Velocity Control " << std::endl;
            returnValue = INCORRECT_MODE;
        } else if (setVelCode != SUCCESS) {
            // Something bad happened
            std::cout << "Joint " << p->getId() << " velocity : " << std::endl;
            ((JointM1 *)p)->errorMessage(setVelCode);
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}

setMovementReturnCode_t RobotM1::applyTorque(JointVec torques) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;  //TODO: proper return error code (not only last one)
    for (auto p : joints) {
        setMovementReturnCode_t setTorCode = ((JointM1 *)p)->setTorque(torques(i));
        if (setTorCode == INCORRECT_MODE) {
            std::cout << "Joint " << p->getId() << ": is not in Torque Control " << std::endl;
            returnValue = INCORRECT_MODE;
        } else if (setTorCode != SUCCESS) {
            // Something bad happened
            std::cout << "Joint " << p->getId() << ": Unknown Error " << std::endl;
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }
    return returnValue;
}

EndEffVec RobotM1::directKinematic(JointVec q_given) {
    Vector2d X;

    X(0) = LinkLengths(0) * cos(q_given(0));
    X(1) = LinkLengths(0) * sin(q_given(0));

    return X;
}

JointVec RobotM1::inverseKinematic(EndEffVec X) {
    // Calculate joint angle from (X,Y)
    JointVec q_exp;
    q_exp(0) = -atan2(X(1), -X(0));
    return q_exp;
}

JacMtx RobotM1::J() {
    JacMtx J;

    //Jacobian matrix elements
    //TODO: Check Jacobian calculations for M1 - what angle will be considered 0
    J(0) = LinkLengths(0) * sin(q(0));
    J(1) = LinkLengths(0) * cos(q(0));

    return J;
}

JointVec RobotM1::calculateGravityTorques() {
    JointVec tau_g;

    //Calculate gravitational torques
    tau_g(0) = LinkMasses(0) * g(0) * CoGLengths(0) * sin(q(0) - Zero2GravityAngle(0));

    return tau_g;
}

JointVec RobotM1::getJointPos() {
    return q*r2d;
}

JointVec RobotM1::getJointVel() {
    return dq*r2d;
}

JointVec RobotM1::getJointTor() {
    return tau;
}

double RobotM1::filter_q(double alpha_q){
    q_filt(0) = alpha_q*q(0)+(1-alpha_q)*q_filt_pre(0);
    q_filt_pre(0) = q_filt(0);
    return q_filt(0);
}

double RobotM1::filter_dq(double alpha_dq){
    dq_filt(0) = alpha_dq*dq(0)+(1-alpha_dq)*dq_filt_pre(0);
    dq_filt_pre(0) = dq_filt(0);
    return dq_filt(0);
}

double RobotM1::filter_tau_s(double alpha_tau_s){
    tau_s_filt(0) = alpha_tau_s*tau_s(0)+(1-alpha_tau_s)*tau_s_filt_pre(0);
    tau_s_filt_pre(0) = tau_s_filt(0);
    return tau_s_filt(0);
}

JointVec& RobotM1::getJointTor_s() {
    tau_sc(0) =  tau_s(0);
    return tau_sc;
}

JointVec& RobotM1::getJointTor_s_filt() {
    return tau_s_filt;
}

//void RobotM1::updateEstimatedAcceleration() {
//
//    // estimate acceleration as derivative of velocity TODO: use IMU data for less delay
//    acc(0) = controlFreq_*(dq(0) - dq_pre(0)); //2 most recent velocity values divided by timestep
//
//    // filter acceleration signal (hardcoded filter of 50 Hz)
//    double alpha = (2*M_PI*50/controlFreq_)/(2*M_PI*50/controlFreq_+1);
//    acc_filt(0) = alpha*acc(0)+(1-alpha)*acc_filt_pre(0);
//    acc_filt_pre(0) = acc_filt(0);
//
//    filteredGeneralizedAccByDerivative_.tail(X2_NUM_JOINTS) = alphaJoint*generalizedAccByDerivative_.tail(X2_NUM_JOINTS) +
//                                                              (1.0 - alphaJoint)*previousFilteredGeneralizedAccByDerivative_.tail(X2_NUM_JOINTS);
//
//    previousFilteredGeneralizedAccByDerivative_ = filteredGeneralizedAccByDerivative_;
//    dq_pre(0) = dq(0);
//
//    estimatedGeneralizedAcceleration_ = filteredGeneralizedAccByDerivative_;
//}

setMovementReturnCode_t RobotM1::setJointPos(JointVec pos_d) {
    return applyPosition(pos_d*d2r);
}

setMovementReturnCode_t RobotM1::setJointVel(JointVec vel_d) {
    return applyVelocity(vel_d*d2r);
}

setMovementReturnCode_t RobotM1::setJointTor(JointVec tor_d) {
    // filter torque commands with previous command
    tau_motor(0) = (tor_d(0)+tau_motor(0)*2.0)/3.0;
    return applyTorque(tor_d);
}

setMovementReturnCode_t RobotM1::setJointTor_comp(JointVec tor, double ffRatio) {
    double tor_ff;
    double vel = dq_filt(0); //dq(0);
    double pos = q(0); //q(0)

    // previous version
    //double tor_s = tau_s_filt(0);
    //if(abs(vel)<velThresh_)
    //{
    //    if(abs(tor_s)>torqueThresh_)
    //    {
    //        tor_ff = f_s_*sign(tor_s) + i_sin_*sin(pos+t_bias_) + i_cos_*cos(pos+t_bias_);
    //    }
    //    else
    //    {
    //        tor_ff = i_sin_*sin(pos+t_bias_) + i_cos_*cos(pos+t_bias_);
    //    }
    //}
    //else
    //{
    //    vel = vel-sign(vel)*0.1;
    //    tor_ff = f_s_*sign(vel) + f_d_*vel + i_sin_*sin(q(0)+t_bias_) + i_cos_*cos(q(0)+t_bias_) + c2_*sqrt(abs(vel))*sign(vel);
    //}

    // check velocity to apply dynamic friction term
    if(abs(vel)<velThresh_)
    {
        double slowCoef = f_s_/velThresh_; // linear region for static friction
        tor_ff = slowCoef*vel + i_sin_*sin(pos+t_bias_) + i_cos_*cos(pos+t_bias_);
    } else {
        tor_ff = f_s_*sign(vel) + f_d_*vel + i_sin_*sin(pos+t_bias_) + i_cos_*cos(pos+t_bias_) + c2_*sqrt(abs(vel))*sign(vel);
    }
    tor(0) = tor(0) + tor_ff*ffRatio;

    // filter command signal
    double alpha = (2*M_PI*motorTorqueCutOff_/controlFreq_)/(2*M_PI*motorTorqueCutOff_/controlFreq_+1);
    filteredMotorTorqueCommand_ = alpha*tor(0)+(1-alpha)*previousFilteredTorqueCommand_;
    previousFilteredTorqueCommand_ = filteredMotorTorqueCommand_;

    JointVec finalMotorTorqueCommand(M1_NUM_JOINTS);
    finalMotorTorqueCommand(0) = filteredMotorTorqueCommand_;

    return setJointTor(finalMotorTorqueCommand);
}

std::string & RobotM1::getRobotName() {
    return robotName_;
}

void RobotM1::setControlFreq(double controlFreq) {
    controlFreq_ = controlFreq;
}

void RobotM1::setVelThresh(double velThresh) {
    if (m1Params.configFlag) {
        velThresh_ = velThresh * d2r;
    }
}

void RobotM1::setTorqueThresh(double torqueThresh) {
    if (m1Params.configFlag) {
        torqueThresh_ = torqueThresh;
    }
}

void RobotM1::setMotorTorqueCutOff(double cutOff) {
    if (m1Params.configFlag) {
        motorTorqueCutOff_ = cutOff;
    }
}

void RobotM1::setMaxTorqueDF(double tau_filt) {
    tau_df_ = tau_filt;
}

void RobotM1::setMaxTorquePF(double tau_filt) {
    tau_pf_ = tau_filt;
}

void RobotM1::setStimDF(double stim_amp) {
    stim_df_ = stim_amp;
}

void RobotM1::setStimPF(double stim_amp) {
    stim_pf_ = stim_amp;
}

void RobotM1::setStimCalibrate(bool stim_calib) {
    stim_calib_ = stim_calib;
}

void RobotM1::setTorqueOffset(double tau_filt) {
    tau_offset_ = tau_filt;
}

void RobotM1::setAngleOffset(double q_offset) {
    q_offset_ = q_offset*d2r; // center angle in radians
}

void RobotM1::setMaxAngleDF(double q_current) {
    q_df_ = q_current*d2r; // maximum dorsiflexion angle in radians
}

void RobotM1::setMaxAnglePF(double q_current) {
    q_pf_ = q_current*d2r; // maximum plantarflexion angle in radians
}

short RobotM1::sign(double val) { return (val > 0) ? 1 : ((val < 0) ? -1 : 0); }