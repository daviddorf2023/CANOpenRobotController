#include "MultiControllerState.h"

double timeval_to_sec_t(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}

void MultiControllerState::entry(void) {

    spdlog::info("Multi Controller State is entered.");

    //Timing
    clock_gettime(CLOCK_MONOTONIC, &initTime);
    lastTime = timeval_to_sec_t(&initTime);

    // Set up dynamic parameter server
    dynamic_reconfigure::Server<CORC::dynamic_paramsConfig>::CallbackType f;
    f = boost::bind(&MultiControllerState::dynReconfCallback, this, _1, _2);
    server_.setCallback(f);

    // Initialize static parameters (if necessary)
    m1Params = robot_->sendRobotParams();
    v_bias = -1 * m1Params.t_bias[0] * 180. / M_PI;
    if (!m1Params.configFlag) {
        // Update PID and feedforward gains from static parameter server
        kp_ = m1Params.kp[0];
        kd_ = m1Params.kd[0];
        ki_ = m1Params.ki[0];
        cut_off = m1Params.int_torque_cutoff_freq[0];
        ffRatio_ = m1Params.ff_ratio[0];
        spk_ = m1Params.spk[0];
        tick_max_ = m1Params.tick_max[0];
    }

    robot_->initTorqueControl();
    robot_->applyCalibration();
    robot_->calibrateForceSensors();
    robot_->tau_spring[0] = 0;   // for ROS publish only

    // Transparency vectors
    q = Eigen::VectorXd::Zero(1);
    dq = Eigen::VectorXd::Zero(1);
    tau = Eigen::VectorXd::Zero(1);
    tau_s = Eigen::VectorXd::Zero(1);
    tau_cmd = Eigen::VectorXd::Zero(1);

    // Initialize parameters
    control_freq = 400.0; //400
    error = 0;
    delta_error = 0;
    integral_error = 0;
    tick_count = 0;
    controller_mode_ = -1;
//    cut_off = 6.0;

    cycle = 0;
    id_mode = 1; // sine = 1; ramp = 2
    counter = 0;
    max_cycle = 4;
    freq = 0;
    mag = 0;
    s_mag = 0;
    s_mag_prev = 0;
    step = 0;
    max_tau = 4;
    dir = true;

    digitalInValue_ = 0;
    digitalOutValue_ = 0; //0
    robot_->setDigitalOut(digitalOutValue_);
    robot_->setControlFreq(control_freq);
}
void MultiControllerState::during(void) {

    //Compute some basic time values
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    double now = timeval_to_sec_t(&ts);
    elapsedTime = (now-timeval_to_sec_t(&initTime));
    dt = now - lastTime;
    lastTime = now;

    //
    tick_count = tick_count + 1;
    if (controller_mode_ == 0) {  // Homing
        if (cali_stage == 1) {
            // set calibration velocity
            JointVec dq_t;
            dq_t(0) = cali_velocity; // calibration velocity
            if (robot_->setJointVel(dq_t) != SUCCESS) {
                std::cout << "Error: " << std::endl;
            }

            // monitor velocity and interaction torque
            dq= robot_->getJointVel();
            //tau = robot_->getJointTor_s(); //TODO: monitor motor torque instead of sensor torque
            tau = robot_->getJointTor();
            if ((dq(0) <= 2) & (tau(0) <= -13)) {
                cali_velocity = 0;
                robot_->applyCalibration();
                robot_->initPositionControl();
                cali_stage = 2;
            } else {
                robot_->printJointStatus();
            }
        } else if (cali_stage == 2) {
            // set position control: 16 is vertical for old M1s, 20 is vertical for new
            q(0) = v_bias; //16
            if(robot_->setJointPos(q) != SUCCESS){
                std::cout << "Error: " << std::endl;
            }
            //if (tau(0)>0.2 || tau(0)<-0.2)
            //{
            //    robot_->m1ForceSensor->calibrate();
            //}
            robot_->m1ForceSensor->calibrate();
            robot_->printJointStatus();
            std::cout << "Calibration done!" << std::endl;
            cali_stage = 3;
        }
    }
    else if (controller_mode_ == 1) {  // zero torque mode
        robot_->setJointTor(Eigen::VectorXd::Zero(M1_NUM_JOINTS));
    }
    else if (controller_mode_ == 2) { // follow position commands
        robot_->setJointPos(multiM1MachineRos_->jointPositionCommand_);
    }
    else if (controller_mode_ == 3) { // follow torque commands
        robot_->setJointTor(multiM1MachineRos_->jointTorqueCommand_);
    }
    else if (controller_mode_ == 4) { // virtual spring - torque mode
        tau = robot_->getJointTor();
        //tau_s = (robot_->getJointTor_s()+tau_s)/2.0;
        dq = robot_->getJointVel();

        // filter position
        q = robot_->getJointPos();
        q_raw = q(0);
        alpha_q = (2*M_PI*dt*cut_off)/(2*M_PI*dt*cut_off+1);
        robot_->filter_q(alpha_q);
        q = robot_->getJointPos();
        q_filtered = q(0);

        // filter torque signal
        tau_s = robot_->getJointTor_s();
        tau_raw = tau_s(0);
        alpha_tau = (2*M_PI*dt*cut_off)/(2*M_PI*dt*cut_off+1);
        robot_->filter_tau(alpha_tau);
        tau_s = robot_->getJointTor_s();
        tau_filtered = tau_s(0);
//        std::cout << "Pre :" << tau_raw << "; Post :" << tau_filtered << std::endl;

        // get interaction torque from virtual spring
        spring_tor = -multiM1MachineRos_->interactionTorqueCommand_(0);
//        spring_tor = spk_*M_PIf64*(45-q(0))/180.0;  //stiffness; q(0) in degree
        robot_->tau_spring[0] = spring_tor;   // for ROS publish only

        // torque tracking with PD controller
        error = tau_s(0) + spring_tor;  // interaction torque error, desired interaction torque is spring_tor, 1.5 is ratio to achieve the desired torque
        delta_error = (error-torque_error_last_time_step)*control_freq;  // derivative of interaction torque error;
        integral_error = integral_error + error/control_freq;
        tau_cmd(0) = error*kp_ + delta_error*kd_ + integral_error*ki_;  // tau_cmd = P*error + D*delta_error; 1 and 0.001
        torque_error_last_time_step = error;
        robot_->setJointTor_comp(tau_cmd, tau_s, ffRatio_);

        // reset integral_error every 1 mins, to be decided
        if(tick_count >= control_freq*60){
            integral_error = 0;
            tick_count = 0;
        }
    }
    else if (controller_mode_ == 5) { // transparency - torque mode
        tau = robot_->getJointTor();
        //tau_s = (robot_->getJointTor_s()+tau_s)/2.0;
        dq = robot_->getJointVel();

        // filter q
        q = robot_->getJointPos();
        q_raw = q(0);
        alpha_q = (2*M_PI*dt*cut_off)/(2*M_PI*dt*cut_off+1);
        robot_->filter_q(alpha_q);
        q = robot_->getJointPos();
        q_filtered = q(0);

//         filter torque signal
        tau_s = robot_->getJointTor_s();
        tau_raw = tau_s(0);
        alpha_tau = (2*M_PI*dt*cut_off)/(2*M_PI*dt*cut_off+1);
        robot_->filter_tau(alpha_tau);
        tau_s = robot_->getJointTor_s();
        tau_filtered = tau_s(0);

        // torque tracking with PID controller
        error = tau_s(0);  // interaction torque error, desired interaction torque is 0
        delta_error = (error-torque_error_last_time_step)*control_freq;  // derivative of interaction torque error;
        integral_error = integral_error + error/control_freq;
        tau_cmd(0) = error*kp_ + delta_error*kd_ + integral_error*ki_;  // tau_cmd = P*error + D*delta_error + ; 1 and 0.001
        torque_error_last_time_step = error;
        robot_->setJointTor_comp(tau_cmd, tau_s, ffRatio_);

        // reset integral_error every 1 mins, to be decided
        if(tick_count >= control_freq*tick_max_){
            integral_error = 0;
            tick_count = 0;
        }
    }
    else if(controller_mode_ == 6) {  // fixed neutral - zero velocity mode
        if (fixed_stage == 1) {
            // set calibration velocity
            JointVec dq_t;
            dq_t(0) = cali_velocity; // calibration velocity
            if (robot_->setJointVel(dq_t) != SUCCESS) {
                std::cout << "Error: " << std::endl;
            }

            // monitor velocity and joint torque
            dq = robot_->getJointVel();
            tau = robot_->getJointTor();
            if ((dq(0) <= 2) & (tau(0) <= -13)) {
                cali_velocity = 0;
                robot_->applyCalibration();
                robot_->initPositionControl();
                fixed_stage = 2;
            } else {
                robot_->printJointStatus();
            }
        }
        if (fixed_stage == 2) {
            // set fixed neutral position
            JointVec q_t;
            q_t(0) = fixed_q;
            if(robot_->setJointPos(q_t) != SUCCESS){
                std::cout << "Error: " << std::endl;
            }

            // monitor position
            q = robot_->getJointPos();
            if (abs(q(0)-fixed_q)<0.001){
                robot_->initVelocityControl();
                std::cout << "Holding neutral position with zero velocity" << std::endl;
                fixed_stage = 3;
            }
            else {
                robot_->printJointStatus();
            }
        } else if (fixed_stage == 3) {
            // apply zero velocity mode
            robot_->setJointVel(Eigen::VectorXd::Zero(M1_NUM_JOINTS));
        }
    }
    else if(controller_mode_ == 7) {  // system identification with torque mode
        counter = counter + 1;
        if(counter%100==1) {
            robot_->printJointStatus();
        }

        // change frequency/magnitude/step amount after max_cycle
        if(cycle >= max_cycle) {
            cycle = 0;
            start = true;
            counter = 0;

            if (id_mode == 1) {
                mag = mag + 0.2; // sine wave magnitude (torque)
                freq = freq + 0.1; // sine wave frequency (torque)
                if (mag > 5.0) {
                    freq = 0.1;
                    mag = 3;
                    id_mode = 2;
                    std::cout << "switch to ramp" << std::endl;
                }
            } else if (id_mode == 2) {
                step = step + 0.2; // slope of ramp (torque)
                if (step > 1.00) {
                    step = 0.1;
                    max_tau = 4;
                    id_mode = 1;
                    std::cout << "switch to sine" << std::endl;
                }
            }
        }

        q = robot_->getJointPos();
        if (id_mode == 1) {
            // set sinusoidal torque input
            if (cycle == 0 && start) {
                start = false;
                std::cout << std::setprecision(2) << "torque magnitude: " << mag << "; frequency: " << freq
                          << "; Cycle: " << cycle << std::endl;
            }
            s_mag = sin(2 * M_PI * freq * counter / control_freq);
            tau_cmd(0) = mag * s_mag + 0.8; //mag * s_mag + 0.8;
            if(s_mag_prev < 0 && s_mag > 0)
            {
                cycle = cycle + 1;
                if (cycle < max_cycle) {
                    std::cout << std::setprecision(2) << "torque magnitude: " << mag << "; frequency: " << freq
                              << "; Cycle: " << cycle << std::endl;
                }
            }
            s_mag_prev = s_mag;
        } else if (id_mode == 2) {
            // set ramp torque input
            if (cycle == 0 && start) {
                start = false;
                std::cout << std::setprecision(2) << "step magnitude: " << step << "; Cycle: " << cycle
                          << std::endl;
            }
            // change torque direction while monitoring angle between 10 and 80 degrees
            if (dir) {
                if (q(0) < 80) {
                    tau_cmd(0) = tau_cmd(0) + step;
                } else {
                    dir = false;
                }
            } else {
                if (q(0) > 10) {
                    tau_cmd(0) = tau_cmd(0) - step;
                } else {
                    dir = true;
                    cycle = cycle + 1;
                    if (cycle < max_cycle) {
                        std::cout << std::setprecision(2) << "step magnitude: " << step << "; Cycle: " << cycle
                                  << std::endl;
                    }
                }
            }
            // set upper and lower limits of ramp torque command
            if (tau_cmd(0) > max_tau) {
                tau_cmd(0) = max_tau;
            } else if (tau_cmd(0) < -1 * max_tau) {
                tau_cmd(0) = -1 * max_tau;
            }
        }
        // set joint torque command
        if (robot_->setJointTor(tau_cmd) != SUCCESS) {
            std::cout << "Error: " << std::endl;
        }
    }
    else if (controller_mode_ == 11) { // Send high for external trigger with EMG

        double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0;

        if(robot_->getRobotName() == m1_trigger){
            if(time > 2.0){
                if (digitalInValue_ == 1) {
                    digitalOutValue_ = 0;
                    robot_->setDigitalOut(digitalOutValue_);
                }
            }
            else if(time > 1.0){
                if (digitalInValue_ == 0) {
                    digitalOutValue_ = 1;
                    robot_->setDigitalOut(digitalOutValue_);
                }
            }
        }
    }
    // Read setDigitalOut signal
    //if(robot_->getRobotName() == m1_trigger){
    //   digitalInValue_ = robot_->getDigitalIn();
    //}
}

void MultiControllerState::exit(void) {
}

void MultiControllerState::dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level) {
    // Update PID and feedforward gains from RQT GUI
    if (m1Params.configFlag) {
        kp_ = config.kp;
        kd_ = config.kd;
        ki_ = config.ki;
        robot_->setVelThresh(config.vel_thresh * M_PI/180.);
        robot_->setMotorTorqueCutOff(config.motor_torque_cutoff_freq);

        cut_off = config.int_torque_cutoff_freq;
        ffRatio_ = config.ff_ratio;
        spk_ = config.spk;

        if(tick_max_ != config.tick_max)
        {
            tick_max_ = config.tick_max;
            tick_count = 0;
        }
    } else {
        std::cout << "Dynamic reconfigure parameter setting is disabled (set configFlag to true to enable)" << std::endl;
    }

    // Change control mode on RQT GUI change
    if(controller_mode_!=config.controller_mode)
    {
        controller_mode_ = config.controller_mode;
        if(controller_mode_ == 0) {
            robot_->initVelocityControl();
            cali_stage = 1;
            cali_velocity = -30;
        }

        cycle = 0;
        counter = 0;

        if (controller_mode_ == 1) robot_->initTorqueControl();
        if (controller_mode_ == 2) robot_->initPositionControl();
        if (controller_mode_ == 3) robot_->initTorqueControl();
        if (controller_mode_ == 4) robot_->initTorqueControl();
        if (controller_mode_ == 5) robot_->initTorqueControl();

        if (controller_mode_ == 6) {
            robot_->initVelocityControl();
            fixed_stage = 1;
            fixed_q = 90 + v_bias; // force calibration = 90+v_bias, trajectory bias = 45
            cali_velocity = -30;
        }
        if (controller_mode_ == 7) {
            robot_->initTorqueControl();
            freq = 0.1;
            mag = 3;   // magnitude for sine wave (without compensation = 3, with compensation = 0.6)
            s_mag = 0; // current value of sine wave
            s_mag_prev = 0; // previous value of sine wave
            step = 0.1; // step for ramp
            dir = true;
            start = true;
        }
        if (controller_mode_ == 11) time0 = std::chrono::steady_clock::now();
        if (controller_mode_ == 11) robot_->setDigitalOut(0);
    }

    return;
}


