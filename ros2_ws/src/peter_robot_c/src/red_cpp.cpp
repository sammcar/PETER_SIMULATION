#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <numeric>


class NetworkPublisher : public rclcpp::Node
{
public:
    NetworkPublisher()
    : Node("network_publisher")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        mode_pub_ = this->create_publisher<std_msgs::msg::String>("/peter_mode", 10);
        gpe_r_pub_ = this->create_publisher<std_msgs::msg::Float64>("/Gpe_Hostil", 10);
        gpe_g_pub_ = this->create_publisher<std_msgs::msg::Float64>("/Gpe_Obstaculo", 10);
        gpe_b_pub_ = this->create_publisher<std_msgs::msg::Float64>("/Gpe_Apetente", 10);


        red_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/bounding_box/red", 100,
            std::bind(&NetworkPublisher::red_callback, this, std::placeholders::_1));

        green_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/bounding_box/green", 100,
            std::bind(&NetworkPublisher::green_callback, this, std::placeholders::_1));

        blue_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/bounding_box/blue", 100,
            std::bind(&NetworkPublisher::blue_callback, this, std::placeholders::_1));
              

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&NetworkPublisher::imu_callback, this, std::placeholders::_1));

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&NetworkPublisher::lidar_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&NetworkPublisher::run_network, this));

        speed_ = 5.0;  turn_ = 5.0;
        max_speed_ = 6.5; min_speed_ = 0.1;
        max_turn_ = 6.5; min_turn_ = 0.1;
        current_mode_ = "C";

        areaBoundingBoxR_ = 0.0; areaBoundingBoxG_ = 0.0; areaBoundingBoxB_ = 0.0;
        posR_ = 0.0; posG_ = 0.0; posB_ = 0.0;

        init_constants();
        RCLCPP_INFO(this->get_logger(), "NetworkPublisher node started.");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gpe_r_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gpe_g_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gpe_b_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr red_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr green_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr blue_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double speed_, turn_, max_speed_, min_speed_, max_turn_, min_turn_;
    std::string current_mode_;
    double areaBoundingBoxR_, areaBoundingBoxG_, areaBoundingBoxB_;
    double posR_, posG_, posB_;

    int ang_p_, ang_s_;
    double epsilem_, dt_, cte_, Area_;
    double roll_, pitch_, std_dev_accel_z_;
    double w_, j_, A_, Sigma_, SigmaIMU_;
    double tau_, tauMotor_, TaoGpi_, TaoGpe_, TaoSTN_, TaoSTR_;
    double Usigma_az_, Upitch_, Uroll_;

    std::vector<std::vector<double>> W_input_to_response_;
    std::vector<std::vector<double>> weights_r_r_;
    std::vector<std::vector<double>> W_response_to_aux_;

    int n_neuronas_, n_por_cuadrante_;
    std::vector<double> activaciones_totales_, Directions_deg_, Directions_rad_, accel_z_history_;
    std::vector<std::vector<double>> Om_;
    double sigma_, umbral_;

    double Gpi_[3][2] = {};
    double Gpe_[3][2] = {};
    double StN_[3][2] = {};
    double StR_[6][2] = {};
    double z_[20][2] = {};
    double lidar_[5][2] = {};
    double Response_[16][2] = {};
    double Aux_[16][2] = {};

    void init_constants()
    {
        ang_p_ = 90; ang_s_ = 90; epsilem_ = 0.01; dt_ = 1.0; cte_ = 1.0; Area_ = 100000.0;
        roll_ = 0.0; pitch_ = 0.0; std_dev_accel_z_ = 0.0;
        w_ = 10.0; j_ = 2.0; A_ = 5.0; Sigma_ = 0.3; SigmaIMU_ = 1.5;
        tau_ = 1.0; tauMotor_ = 2.0; TaoGpi_ = 1.0; TaoGpe_ = 2.0; TaoSTN_ = 2.0; TaoSTR_ = 1.0;
        Usigma_az_ = 30.0; Upitch_ = 15.0; Uroll_ = 15.0;

        W_input_to_response_.resize(16, std::vector<double>(16, 0.0));
        weights_r_r_.resize(16, std::vector<double>(16, -1.0));
        W_response_to_aux_.resize(16, std::vector<double>(16, 0.0));
        for (int i = 0; i < 16; ++i) {
            W_input_to_response_[i][15 - i] = 1.0;
            weights_r_r_[i][i] = 0.0;
            for (int j = i; j < 16; ++j) {
                W_response_to_aux_[i][j] = 1.0;
            }
        }

        n_neuronas_ = 16; n_por_cuadrante_ = n_neuronas_ / 4;
        activaciones_totales_.resize(n_neuronas_, 0.0);
        sigma_ = 0.06; umbral_ = 0.95;
        Directions_deg_.resize(n_neuronas_);
        for (int i = 0; i < n_por_cuadrante_; ++i) {
            Directions_deg_[i] = i * (90.0 / n_por_cuadrante_);
            Directions_deg_[i + n_por_cuadrante_] = 90.0 + i * (90.0 / n_por_cuadrante_);
            Directions_deg_[i + 2 * n_por_cuadrante_] = 180.0 + i * (90.0 / n_por_cuadrante_);
            Directions_deg_[i + 3 * n_por_cuadrante_] = 270.0 + i * (90.0 / n_por_cuadrante_);
        }
        Directions_rad_.resize(n_neuronas_);
        Om_.resize(2, std::vector<double>(n_neuronas_));
        for (int i = 0; i < n_neuronas_; ++i) {
            Directions_rad_[i] = Directions_deg_[i] * M_PI / 180.0;
            Om_[0][i] = std::cos(Directions_rad_[i]);
            Om_[1][i] = std::sin(Directions_rad_[i]);
        }
    }

    double gausiana(const std::vector<double>& theta, const std::vector<double>& omega)
    {
        double dot_product = theta[0] * omega[0] + theta[1] * omega[1];
        return std::exp((dot_product - 1.0) / (2.0 * sigma_ * sigma_));
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        activaciones_totales_.assign(n_neuronas_, 0.0);
        double r_min = 0.2;
        double r_max = 1.0;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double r = msg->ranges[i];
            double angle = msg->angle_min + i * msg->angle_increment;
            if (std::isnan(r) || std::isinf(r) || r < r_min || r > r_max) {
                continue;
            }
            std::vector<double> vector_input = {std::cos(angle), std::sin(angle)};
            for (int j = 0; j < n_neuronas_; ++j) {
                std::vector<double> omega = {Om_[0][j], Om_[1][j]};
                double activation = gausiana(vector_input, omega) * 130.0 / (1000.0 * r);
                activaciones_totales_[j] = std::max(activaciones_totales_[j], activation);
            }
        }
    }

    void publish_twist(double linear_x = 0.0, double linear_y = 0.0, double angular_z = 0.0)
    {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = linear_x;
        twist.linear.y = linear_y;
        twist.angular.z = angular_z;
        cmd_vel_pub_->publish(twist);
    }

    void publish_mode(const std::string & mode)
    {
        std_msgs::msg::String mode_msg;
        mode_msg.data = mode;
        mode_pub_->publish(mode_msg);
        current_mode_ = mode;
    }

    void publish_gpe(double gper = 0.0, double gpeg = 0.0, double gpeb = 0.0)
    {
        std_msgs::msg::Float64 msg_r;
        std_msgs::msg::Float64 msg_g;
        std_msgs::msg::Float64 msg_b;

        msg_r.data = gper;
        msg_g.data = gpeg;
        msg_b.data = gpeb;

        gpe_r_pub_->publish(msg_r);
        gpe_g_pub_->publish(msg_g);
        gpe_b_pub_->publish(msg_b);
    }

    void green_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 2) {
            posG_ = msg->data[0];
            areaBoundingBoxG_ = msg->data[1];
        }
    }

    void blue_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 2) {
            posB_ = msg->data[0];
            areaBoundingBoxB_ = msg->data[1];
        }
    }

    void red_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 2) {
            posR_ = msg->data[0];
            areaBoundingBoxR_ = msg->data[1];
        }
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;

        roll_ = 180.0 - std::abs(std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)) * 180.0 / M_PI);
        pitch_ = std::abs(std::asin(2.0 * (qw * qy - qz * qx)) * 180.0 / M_PI);

        accel_z_history_.push_back(msg->linear_acceleration.z);
        if (accel_z_history_.size() > 1) {
            double mean = std::accumulate(accel_z_history_.begin(), accel_z_history_.end(), 0.0) / accel_z_history_.size();
            double sq_sum = std::inner_product(accel_z_history_.begin(), accel_z_history_.end(), accel_z_history_.begin(), 0.0);
            std_dev_accel_z_ = std::sqrt(sq_sum / accel_z_history_.size() - mean * mean);
        } else {
            std_dev_accel_z_ = 0.0;
        }
    }

    void run_network()
{
    // Variables de comando
    double cmd_lineal = 0.0;
    double cmd_ang = 0.0;
    double cmd_lateral = 0.0;

    //--------------- Dinámica del sensor LIDAR ----------------
    lidar_[0][1] = lidar_[0][0] + (dt_ / 10.0) * (
        -lidar_[0][0] + (
            std::accumulate(activaciones_totales_.begin(), activaciones_totales_.begin() + 4, 0.0) +
            std::accumulate(activaciones_totales_.begin() + 12, activaciones_totales_.begin() + 15, 0.0) -
            lidar_[1][0]
        )
    );
    lidar_[1][1] = lidar_[1][0] + (dt_ / 10.0) * (
        -lidar_[1][0] + (
            std::accumulate(activaciones_totales_.begin() + 4, activaciones_totales_.begin() + 12, 0.0) -
            lidar_[0][0]
        )
    );
    lidar_[2][1] = lidar_[2][0] + (dt_ / 10.0) * (
        -lidar_[2][0] + (
            std::accumulate(activaciones_totales_.begin(), activaciones_totales_.begin() + 8, 0.0) -
            lidar_[3][0]
        )
    );
    lidar_[3][1] = lidar_[3][0] + (dt_ / 10.0) * (
        -lidar_[3][0] + (
            std::accumulate(activaciones_totales_.begin() + 8, activaciones_totales_.begin() + 15, 0.0) -
            lidar_[2][0]
        )
    );

    //--------------- Dinámica de Response y Aux ----------------
    // Response & Aux dynamics
    for (int r=0;r<16;++r) {
        double sum_in=0.0, sum_rr=0.0;
        for (int c=0;c<16;++c) sum_in    += W_input_to_response_[r][c] * activaciones_totales_[c];
        for (int c=0;c<16;++c) sum_rr    += weights_r_r_[r][c]          * Response_[c][0];
        double net_r = sum_in + sum_rr;
        Response_[r][1] = Response_[r][0] + (dt_/5.0)*(-Response_[r][0] + std::max(0.0,net_r));

        double sum_ra = 0.0;
        for (int c=0;c<16;++c) sum_ra += W_response_to_aux_[r][c] * Response_[c][1];
        Aux_[r][1] = Aux_[r][0] + (dt_/5.0)*(-Aux_[r][0] + std::max(0.0,sum_ra));
    }
    double sum_aux=0.0; for(int i=0;i<16;++i) sum_aux+=Aux_[i][0];
    lidar_[4][1] = lidar_[4][0] + (dt_/tau_)*(-lidar_[4][0] + std::max(0.0,sum_aux));

    //--------------- Cálculo de estímulos R, G, B ----------------
    double R = areaBoundingBoxR_ / 500.0;
    double G = (lidar_[4][0] * 10.0 > epsilem_) ? lidar_[4][0] * 10.0 : 0.0;
    double B = areaBoundingBoxB_ / 500.0;

    //--------------- Núcleo basal (StN, Gpi, Gpe, StR) ----------------
    StN_[0][1] = std::max(0.0,
        StN_[0][0] + (1.0 / TaoSTN_) * (-5.0 * StN_[0][0] + R - Gpi_[0][0] - Gpe_[1][0] - Gpe_[2][0] - 1.0)
    );
    StN_[1][1] = std::max(0.0,
        StN_[1][0] + (1.0 / TaoSTN_) * (-5.0 * StN_[1][0] + G - Gpi_[1][0] - Gpe_[0][0] - Gpe_[2][0] - 1.0)
    );
    StN_[2][1] = std::max(0.0,
        StN_[2][0] + (1.0 / TaoSTN_) * (-5.0 * StN_[2][0] + B - Gpi_[2][0] - Gpe_[0][0] - Gpe_[1][0] - 1.0)
    );

    Gpi_[0][1] = std::max(0.0,
        Gpi_[0][0] + (1.0 / TaoGpi_) * (-Gpi_[0][0] + StN_[1][0] + StN_[2][0] - Gpe_[0][0] - StR_[0][0])
    );
    Gpi_[1][1] = std::max(0.0,
        Gpi_[1][0] + (1.0 / TaoGpi_) * (-Gpi_[1][0] + StN_[0][0] + StN_[2][0] - Gpe_[1][0] - StR_[1][0])
    );
    Gpi_[2][1] = std::max(0.0,
        Gpi_[2][0] + (1.0 / TaoGpi_) * (-Gpi_[2][0] + StN_[0][0] + StN_[1][0] - Gpe_[2][0] - StR_[2][0])
    );

    Gpe_[0][1] = std::max(0.0,
        Gpe_[0][0] + (1.0 / TaoGpe_) * (-Gpe_[0][0] + StN_[0][0])
    );
    Gpe_[1][1] = std::max(0.0,
        Gpe_[1][0] + (1.0 / TaoGpe_) * (-Gpe_[1][0] + StN_[1][0])
    );
    Gpe_[2][1] = std::max(0.0,
        Gpe_[2][0] + (1.0 / TaoGpe_) * (-Gpe_[2][0] + StN_[2][0])
    );

    StR_[0][1] = std::max(0.0,
        StR_[0][0] + (1.0 / TaoSTR_) * (-StR_[0][0] + StN_[0][0])
    );
    StR_[1][1] = std::max(0.0,
        StR_[1][0] + (1.0 / TaoSTR_) * (-StR_[1][0] + StN_[1][0])
    );
    StR_[2][1] = std::max(0.0,
        StR_[2][0] + (1.0 / TaoSTR_) * (-StR_[2][0] + StN_[2][0])
    );

    //--------------- Ángulo de estímulo ----------------
    if (Gpe_[0][1] > 1.5 && R > 0.5) {
        ang_s_ = posR_;
    } else if (Gpe_[1][1] > 1.5 && G > 0.5) {
        ang_s_ = (lidar_[0][0] > 0.5 ? 90.0 : 0.0) +
                 (lidar_[2][0] > 0.5 ? 170.0 : 0.0) +
                 (lidar_[3][0] > 0.5 ? 10.0 : 0.0);
    } else if (Gpe_[2][1] > 1.5 && B > 0.5) {
        ang_s_ = posB_;
    } else {
        ang_s_ = 90.0;
    }

    //--------------- Dinámica IMU ----------------
    z_[0][1] = z_[0][0] + (dt_ / tau_) * (
        -z_[0][0] + (A_ * std::pow(std::max(0.0, std_dev_accel_z_ - Usigma_az_), 2)) /
        (std::pow(SigmaIMU_, 2) + std::pow(-z_[0][0] + std_dev_accel_z_ - Usigma_az_, 2))
    );
    z_[1][1] = z_[1][0] + (dt_ / tau_) * (
        -z_[1][0] + (A_ * std::pow(std::max(0.0, pitch_ - Upitch_), 2)) /
        (std::pow(SigmaIMU_, 2) + std::pow(-z_[1][0] + pitch_ - Upitch_, 2))
    );
    z_[2][1] = z_[2][0] + (dt_ / tau_) * (
        -z_[2][0] + (A_ * std::pow(std::max(0.0, roll_ - Uroll_), 2)) /
        (std::pow(SigmaIMU_, 2) + std::pow(-z_[2][0] + roll_ - Uroll_, 2))
    );
    z_[3][1] = z_[3][0] + (dt_ / tau_) * (-z_[3][0] + std::max(0.0, Gpe_[2][0]));
    z_[4][1] = z_[4][0] + (dt_ / tau_) * (-z_[4][0] + std::max(0.0, Gpe_[1][0] + j_ * Gpe_[0][0]));
    z_[5][1] = z_[5][0] + (dt_ / tau_) * (-z_[5][0] + std::max(0.0, ang_s_ - ang_p_ - 20.0));
    z_[6][1] = z_[6][0] + (dt_ / tau_) * (-z_[6][0] + std::max(0.0, ang_p_ - ang_s_ - 20.0));
    z_[7][1] = z_[7][0] + (dt_ / tau_) * (-z_[7][0] + std::max(0.0, z_[5][0] + z_[3][0] - w_ * z_[4][0]));
    z_[8][1] = z_[8][0] + (dt_ / tau_) * (-z_[8][0] + std::max(0.0, z_[5][0] + z_[4][0] - w_ * z_[3][0]));
    z_[9][1] = z_[9][0] + (dt_ / tau_) * (-z_[9][0] + std::max(0.0, z_[4][0] + z_[6][0] - w_ * z_[3][0]));
    z_[10][1] = z_[10][0] + (dt_ / tau_) * (-z_[10][0] + std::max(0.0, z_[3][0] + z_[6][0] - w_ * z_[4][0]));
    z_[11][1] = z_[11][0] + (dt_ / tau_) * (-z_[11][0] + std::max(0.0, z_[7][0] + z_[9][0]));
    z_[12][1] = z_[12][0] + (dt_ / tau_) * (-z_[12][0] + std::max(0.0, z_[10][0] + z_[8][0]));
    z_[13][1] = z_[13][0] + (dt_ / tau_) * (-z_[13][0] + std::max(0.0, -w_ * std::abs(cmd_ang) * z_[11][0] - w_ * std::abs(cmd_ang) * z_[12][0] - w_ * z_[17][0] + cte_));
    z_[14][1] = z_[14][0] + (dt_ / tau_) * (-z_[14][0] + (A_ * std::pow(std::max(0.0, 100.0 * Gpe_[1][0] - w_ * Gpi_[0][0] - w_ * Gpi_[2][0] - w_ * z_[15][0] - w_ * z_[16][0]), 2)) /
        (std::pow(Sigma_, 2) + std::pow(100.0 * Gpe_[1][0] - w_ * Gpi_[0][0] - w_ * Gpi_[2][0] - w_ * z_[15][0] - w_ * z_[16][0], 2)));
    z_[15][1] = z_[15][0] + (dt_ / tau_) * (-z_[15][0] + (A_ * std::pow(std::max(0.0, -Gpe_[1][0] - cte_ + w_ * z_[3][0] + 0.7 * z_[0][0] + 0.7 * z_[1][0] + 0.7 * z_[2][0] - w_ * z_[14][0] * 1.5 - w_ * z_[16][0]), 2)) /
        (std::pow(Sigma_, 2) + std::pow(-Gpe_[1][0] - 0.5 * cte_ + 2.0 * z_[3][0] + 20.0 * z_[0][0] + 0.7 * z_[1][0] + 0.7 * z_[2][0] - w_ * z_[14][0] * 1.5 - w_ * z_[16][0], 2)));
    z_[16][1] = z_[16][0] + (dt_ / tau_) * (-z_[16][0] + (A_ * std::pow(std::max(0.0, z_[4][0] - w_ * Gpe_[1][0] - w_ * z_[14][0] * 1.5 - w_ * z_[15][0] * 1.5 + cte_), 2)) /
        (std::pow(Sigma_, 2) + std::pow(z_[4][0] - w_ * Gpe_[1][0] - w_ * z_[14][0] * 1.5 - w_ * z_[15][0] * 1.5 + cte_, 2)));
    z_[17][1] = z_[17][0] + (dt_ / tau_) * (-z_[17][0] + std::max(0.0, Gpe_[2][0] - Area_));

    //--------------- Cálculo de comandos ----------------
    cmd_ang = (z_[11][0] * (Gpe_[1][0] < 0.5)) - (z_[12][0] * (Gpe_[1][0] < 0.5));
    cmd_lateral = (-lidar_[2][0] * 1.5 + lidar_[3][0] * 1.5 + z_[11][0] * (Gpe_[1][0] > 0.5)) - (z_[12][0] * (Gpe_[1][0] > 0.5));
    cmd_lineal = -lidar_[0][0] * 1.5 + lidar_[1][0] * 1.5 + z_[13][0] - j_ * z_[4][0] * ((z_[5][0] < epsilem_) && (z_[6][0] < epsilem_));

    //--------------- Actualizar estados ----------------
    for (int i = 0; i < 18; ++i) {
        z_[i][0] = (z_[i][1] > epsilem_) ? z_[i][1] : 0.0;
    }
    for (int i = 0; i < 5; ++i) {
        lidar_[i][0] = (lidar_[i][1] > epsilem_) ? lidar_[i][1] : 0.0;
    }
    for (int i = 0; i < 16; ++i) {
        Response_[i][0] = (Response_[i][1] > epsilem_) ? Response_[i][1] : 0.0;
        Aux_[i][0] = (Aux_[i][1] > epsilem_) ? Aux_[i][1] : 0.0;
    }
    for (int i = 0; i < 3; ++i) {
        StN_[i][0] = (StN_[i][1] > epsilem_) ? StN_[i][1] : 0.0;
        Gpi_[i][0] = (Gpi_[i][1] > epsilem_) ? Gpi_[i][1] : 0.0;
        Gpe_[i][0] = (Gpe_[i][1] > epsilem_) ? Gpe_[i][1] : 0.0;
    }
    for (int i = 0; i < 3; ++i) {
        StR_[i][0] = (StR_[i][1] > epsilem_) ? StR_[i][1] : 0.0;
    }

    //--------------- Publicación ----------------
    publish_gpe(Gpe_[0][1], Gpe_[1][1], Gpe_[2][1]);
    if (epsilem_ < cmd_ang) {
        publish_twist(0.0, 0.0, turn_);
    } else if (cmd_ang < -epsilem_) {
        publish_twist(0.0, 0.0, -turn_);
    } else {
        if (epsilem_ < cmd_lineal) {
            publish_twist(speed_, 0.0, 0.0);
        } else if (cmd_lineal < -epsilem_) {
            publish_twist(-speed_, 0.0, 0.0);
        } else if (z_[17][1] > 0.5) {
            publish_twist(0.0, 0.0, 0.0);
        }
        if (epsilem_ < cmd_lateral) {
            publish_twist(0.0, speed_, 0.0);
        } else if (cmd_lateral < -epsilem_) {
            publish_twist(0.0, -speed_, 0.0);
        }
    }

    if (z_[15][1] > 0.5) {
        publish_mode("C");
    } else if (z_[16][1] > 0.5) {
        publish_mode("H");
    } else if (z_[14][1] > 0.5) {
        publish_mode("X");
    }
}


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NetworkPublisher>());
    rclcpp::shutdown();
    return 0;
}
