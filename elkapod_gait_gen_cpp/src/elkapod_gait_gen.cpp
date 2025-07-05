#include "../include/elkapod_gait_gen_cpp/elkapod_gait_gen.hpp"

static bool is_close(double a, double b, double atol = 1e-8, double rtol = 1e-5) {
    return std::abs(a - b) <= (atol + rtol * std::abs(b));
}

static Eigen::Matrix3d rotZ(double theta){
    Eigen::Matrix3d m;
    m << std::cos(theta), -std::sin(theta), 0.,
        std::sin(theta),  std::cos(theta), 0.,
        0.,               0.,              1.;
    return m;
}

ElkapodGaitGen::ElkapodGaitGen(): Node("elkapod_gait")
{
    trajectory_frequency_ = this->declare_parameter<double>("trajectory_frequency", 50.0);
    min_cycle_time_ = this->declare_parameter<double>("min_cycle_time", 0.5);
    leg_spacing_ = this->declare_parameter<double>("leg_spacing", 0.175);
    step_length_ = this->declare_parameter<double>("step_length", 0.2);
    step_height_ = this->declare_parameter<double>("step_height", 0.1);
    phase_lag_ = this->declare_parameter<double>("phase_lag", 0.03);

    // Subscriptions
    velocity_sub_ = this->create_subscription<VelCmd>(
        "/cmd_vel", 10, std::bind(&ElkapodGaitGen::velocityCallback, this, std::placeholders::_1));

    param_sub_ = this->create_subscription<FloatMsg>(
        "/cmd_base_param", 10, std::bind(&ElkapodGaitGen::paramCallback, this, std::placeholders::_1));

    gait_type_sub_ = this->create_subscription<IntMsg>(
        "/cmd_gait_type", 10, std::bind(&ElkapodGaitGen::gaitTypeCallback, this, std::placeholders::_1));

    // Publishers
    leg_signal_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/elkapod_leg_positions", 10);
    leg_phase_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("leg_phase_signal", 10);

    // Services
    enable_srv_ = this->create_service<ServiceTriggerSrv>(
        "/gait_gen_enable", std::bind(&ElkapodGaitGen::enableServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    disable_srv_ = this->create_service<ServiceTriggerSrv>(
        "/gait_gen_disable", std::bind(&ElkapodGaitGen::disableServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Timer (not started yet)
    leg_clock_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / trajectory_frequency_),
        std::bind(&ElkapodGaitGen::legClockCallback, this));
    leg_clock_timer_->cancel();
    base_traj_ = std::make_unique<ElkapodLegPath>(step_length_, step_height_);

    base_link_rotations_ = {0.63973287, -0.63973287, M_PI/2., -M_PI/2., 2.38414364, -2.38414364};
    base_link_translations_ = {{0.17841, 0.13276, -0.03}, {0.17841, -0.13276, -0.03}, {0.0138, 0.1643, -0.03}, {0.0138, -0.1643, -0.03}, {-0.15903, 0.15038, -0.03}, {-0.15903, -0.15038, -0.03}};
    
    base_height_ = 0.17;
    set_base_height_ = 0.17;
    leg_clock_ = std::vector<double>(6, 0.);
    leg_phase_ = std::vector<int>(6, 0);
    leg_phase_shift_ = std::vector<double>(6, 0.);

    current_vel_ = Eigen::Vector3d::Zero();
    current_vel_scalar_ = 0.;
    current_angular_velocity_ = 0.;
    current_base_direction_ = 0.;

    current_velocity_ = std::vector<Eigen::Vector2d>(6, {0.0, 0.0});
    last_leg_position_ = std::vector<Eigen::Vector3d>(6, {0.0, 0.0, 0.0});
    

    RCLCPP_INFO(this->get_logger(), "Elkapod gait generator initialized. Use service to activate.");
}

void ElkapodGaitGen::init()
{
    base_traj_->init();
    changeGait(gait_type_);
    leg_clock_timer_->reset();
    init_time_ = this->now();
    state_ = State::IDLE;
    RCLCPP_INFO(this->get_logger(), "Elkapod gait generator started.");
}

void ElkapodGaitGen::deinit()
{
    leg_clock_timer_->cancel();
    state_ = State::DISABLED;
}

void ElkapodGaitGen::enableServiceCallback(ServiceTriggerSrv_Req request, ServiceTriggerSrv_Resp response)
{
    if (state_ == State::DISABLED) {
        init();
        response->success = true;
    }
    response->success = false;
    response->message = "Already enabled.";
}

void ElkapodGaitGen::disableServiceCallback(ServiceTriggerSrv_Req request, ServiceTriggerSrv_Resp response)
{
    if (state_ == State::DISABLED) {
        response->success = false;
        response->message = "Already disabled.";
    } else if (state_ == State::WALKING) {
        response->success = false;
        response->message = "Cannot disable while walking.";
    } else {
        deinit();
        response->success = true;
    }
}

void ElkapodGaitGen::gaitTypeCallback(const IntMsg::SharedPtr msg){
    if(state_ == State::IDLE){
        if(msg->data == 0){
            gait_type_ = GaitType::WAVE;
            changeGait(gait_type_);
            RCLCPP_INFO(this->get_logger(), "Gait set to TRIPOD");
        }
        else if(msg->data == 1){
            gait_type_ = GaitType::TRIPOID;
            changeGait(gait_type_);
            RCLCPP_INFO(this->get_logger(), "Gait set to TRIPOD");
        }
    }
}

void ElkapodGaitGen::paramCallback(const FloatMsg::SharedPtr msg){
    if(msg->data > 0.1 && msg->data < 0.2){
        set_base_height_ = msg->data;
    }
    else{
        RCLCPP_WARN(this->get_logger(), "Couldn't set new base height goal - value out of allowed range");
    }
}

void ElkapodGaitGen::velocityCallback(const VelCmd::SharedPtr msg){
    current_vel_[0] = msg->linear.x;
    current_vel_[1] = msg->linear.y;
    current_vel_[2] = msg->angular.z;

    const double vel = Eigen::Vector2d({msg->linear.x, msg->linear.y}).norm();
    const double direction = atan2(msg->linear.y, msg->linear.x);

    if(vel != 0.0 && (vel != current_vel_scalar_ || direction != current_base_direction_)){
        const double T = step_length_ / vel;
        if(min_cycle_time_ >= T){
            RCLCPP_WARN(this->get_logger(), "Provided velocity command exceeds maximum velocity!");
        }
        else{
            RCLCPP_INFO(this->get_logger(), "New velocity command received!");
            current_base_direction_ = direction;
            for(size_t i = 0; i < 6; ++i){
                current_velocity_[i] = {msg->linear.x, msg->linear.y};
            }
            current_vel_scalar_ = vel;
            current_angular_velocity_ = 0.;
            cycle_time_ = T;
            std::fill(leg_phase_shift_.begin(), leg_phase_shift_.end(), 0.);
            changeGait(gait_type_);

        }
    }
    else if(msg->angular.z != 0.0 && !is_close(current_angular_velocity_, msg->angular.z)){
        for(size_t i = 0; i < 6; ++i){
            current_velocity_[i] = {0., 0.};
        }
        current_vel_scalar_ = vel;
        current_angular_velocity_ = msg->angular.z;

        // TODO remove this mysterious 0.22 value and replace it with variable with descriptive name
        const double R = leg_spacing_ + 0.22;
        cycle_time_ = step_length_ / (0.5 * R * fabs(msg->angular.z));
        std::fill(leg_phase_shift_.begin(), leg_phase_shift_.end(), 0.);
        changeGait(gait_type_);
    }

    if(current_vel_.isZero() && state_ == State::WALKING){
        RCLCPP_INFO(this->get_logger(), "Going to IDLE state");
        state_ = State::IDLE;
    }
    else if(!current_vel_.isZero() && state_ == State::IDLE){
        RCLCPP_INFO(this->get_logger(), "Going to WALKING state");
        init_time_ = this->now();
        state_ = State::WALKING;
        std::fill(leg_phase_shift_.begin(), leg_phase_shift_.end(), 0.);
    }
}


void ElkapodGaitGen::changeGait(GaitType gait_type){
    if(gait_type == GaitType::TRIPOID){
        swing_percentage_ = 0.5;

        phase_offset_ = {0.0, cycle_time_ / 2, cycle_time_ / 2, 0.0, 0.0, cycle_time_ / 2};
    }
    else if(gait_type == GaitType::WAVE){
        swing_percentage_ = 1/6.;
        
        std::transform(phase_offset_.begin(), phase_offset_.end(), phase_offset_.begin(),
               [i = 0, this](double) mutable { return i++ * cycle_time_ / 6.0;});

    }
}

void ElkapodGaitGen::clockFunction(double t, double T, double phase_shift, int leg_nb){
    if(!is_close(leg_phase_shift_[leg_nb], phase_shift, 1e-3)){
        if(leg_phase_shift_[leg_nb] < phase_shift){
            leg_phase_shift_[leg_nb] += 0.05 * phase_shift;
        }
        else{
            leg_phase_shift_[leg_nb] -= 0.05 * phase_shift;
        }
    }

    const double t_mod = std::fmod((t + 3*T/4. + leg_phase_shift_[leg_nb]), T);

    if(t_mod < T * phase_lag_){                                         // swing lag phase
        leg_phase_[leg_nb] = 1;
        leg_clock_[leg_nb] = 0.0;
    }
    else if(T * phase_lag_ < t_mod && t_mod < T * swing_percentage_){   // swing
        leg_phase_[leg_nb] = 1;
        leg_clock_[leg_nb] = (t_mod - T * phase_lag_) / (T * swing_percentage_ - T * phase_lag_);
    }
    else{                                                               // stance
        leg_phase_[leg_nb] = 0;
        leg_clock_[leg_nb] = (t_mod - T * swing_percentage_) / (T - T * swing_percentage_);
    }
}

void ElkapodGaitGen::legClockCallback(){

    auto msg_phase = FloatArrayMsg();
    msg_phase.data.resize(6, 0.0);

    if (state_ == State::IDLE) {
        for (int leg_nb = 0; leg_nb < 6; ++leg_nb) {
            leg_phase_[leg_nb] = 0.0;
            leg_clock_[leg_nb] = 0.0;
        }
    } else {
        rclcpp::Time now = get_clock()->now();
        double elapsed_time_sec = (now - init_time_).nanoseconds() / 1e9;

        for (int leg_nb = 0; leg_nb < 6; ++leg_nb) {
            clockFunction(elapsed_time_sec, cycle_time_, phase_offset_[leg_nb], leg_nb);
            msg_phase.data[leg_nb] = leg_phase_[leg_nb];
        }
    }

    leg_phase_pub_->publish(msg_phase);


    auto msg3 = FloatArrayMsg();
    msg3.data.resize(18, 0.0);

    // Base height smoothing
    if (!is_close(base_height_, set_base_height_, 1e-3)) {
        if (base_height_< set_base_height_)
            base_height_ += 0.005;
        else
            base_height_ -= 0.005;
    }



    for (int leg_nb = 0; leg_nb < 6; ++leg_nb) {
        Eigen::Vector3d p;

        if (state_ == State::WALKING) {
            p = (*base_traj_)(leg_clock_[leg_nb], leg_phase_[leg_nb]);


            const double omega = current_angular_velocity_;
            Eigen::Vector2d last_leg_xy(last_leg_position_[leg_nb][0], last_leg_position_[leg_nb][1]);
            Eigen::Vector2d angular_part = omega * Eigen::Vector2d(-last_leg_xy[1], last_leg_xy[0]);

            Eigen::Vector2d vel2D = current_velocity_[leg_nb] + angular_part;

            double angle = std::atan2(vel2D[1], vel2D[0]);
            p = rotZ(angle) * p;
        } else {
            p = Eigen::Vector3d::Zero();
        }

        p = rotZ(-base_link_rotations_[leg_nb]) * p;

        p += Eigen::Vector3d(leg_spacing_, 0.0, -base_height_);

        double rot = base_link_rotations_[leg_nb];
        Eigen::Vector3d trans = base_link_translations_[leg_nb];

        Eigen::Matrix4d L_B_H = Eigen::Matrix4d::Identity();
        L_B_H(0, 0) = std::cos(rot);
        L_B_H(0, 1) = -std::sin(rot);
        L_B_H(1, 0) = std::sin(rot);
        L_B_H(1, 1) = std::cos(rot);
        L_B_H(0, 3) = -(std::cos(rot) * leg_spacing_ + trans[0]);
        L_B_H(1, 3) = -(std::sin(rot) * leg_spacing_ + trans[1]);
        L_B_H(2, 3) = base_height_;

        Eigen::Vector4d p_homogeneous(p[0], p[1], p[2], 1.0);
        Eigen::Vector4d p_base_homogeneous = L_B_H * p_homogeneous;

        last_leg_position_[leg_nb] = p_base_homogeneous.head<3>();

        msg3.data[leg_nb * 3 + 0] = p[0];
        msg3.data[leg_nb * 3 + 1] = p[1];
        msg3.data[leg_nb * 3 + 2] = p[2];
    }

    leg_signal_pub_->publish(msg3);
}