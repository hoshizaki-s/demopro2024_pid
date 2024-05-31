#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/Twist.h"

class PIDController
{
public:
    PIDController(double p, double i, double d)
        : kp_(p), ki_(i), kd_(d), prev_error_(0.0), integral_(0.0) {}

    double compute(double setpoint, double pv, double dt)
    {
        double error = setpoint - pv;
        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

private:
    double kp_;
    double ki_;
    double kd_;
    double prev_error_;
    double integral_;
};

class DeviationController
{
public:
    DeviationController()
        : pid_(0.1, 0.01, 0.05) // PID parameters: P, I, D
    {
        pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        sub_ = nh_.subscribe("/deviation", 10, &DeviationController::deviationCallback, this);
        last_time_ = ros::Time::now();
    }

    void deviationCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        ros::Time now = ros::Time::now();
        double dt = (now - last_time_).toSec();
        last_time_ = now;

        double setpoint = 0.0;  // Desired value
        double cmd_vel = pid_.compute(setpoint, msg->data, dt);
        
        geometry_msgs::Twist twist;
        twist.linear.x = cmd_vel;
        twist.angular.z = 0.0;
        pub_.publish(twist);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    PIDController pid_;
    ros::Time last_time_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "deviation_controller");
    DeviationController controller;
    ros::spin();
    return 0;
}
