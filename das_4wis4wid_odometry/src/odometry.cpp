#include <memory>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "std_srvs/srv/trigger.hpp"

class OdometryPublisher : public rclcpp::Node
{
  public:
    OdometryPublisher()
    : Node("odometry_4wis4wid_node")
    {
      subscription_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",      
            10,                   
            std::bind(&OdometryPublisher::joint_state_callback, this, std::placeholders::_1)
      );
      publisher_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("das/odom", 10);

      // parameters
      this->declare_parameter("x_w_1", 0.1125);
      this->declare_parameter("y_w_1", 0.1125);
      this->declare_parameter("r_w_1", 0.0254);

      this->declare_parameter("x_w_2", -0.1125);
      this->declare_parameter("y_w_2", 0.1125);
      this->declare_parameter("r_w_2", 0.0254);

      this->declare_parameter("x_w_3", -0.1125);
      this->declare_parameter("y_w_3", -0.1125);
      this->declare_parameter("r_w_3", 0.0254);

      this->declare_parameter("x_w_4", 0.1125);
      this->declare_parameter("y_w_4", -0.1125);
      this->declare_parameter("r_w_4", 0.0254);
    }

  private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      //mapping joint names with values
      if (joint_index_.empty()) {
          for (size_t i = 0; i < msg->name.size(); ++i) {
              joint_index_[msg->name[i]] = i;
          }
          // initialize last_time_ when we receive the first message
          last_time_ = this->get_clock()->now();
          return;
      }

      // compute dt
      rclcpp::Time current_time = this->get_clock()->now();
      double dt = (current_time - last_time_).seconds();
      last_time_ = current_time;
      
      //robot dimensions
      double x_w[4] = {this->get_parameter("x_w_1").as_double(), this->get_parameter("x_w_2").as_double(), this->get_parameter("x_w_3").as_double(), this->get_parameter("x_w_4").as_double()};
      double y_w[4] = {this->get_parameter("y_w_1").as_double(), this->get_parameter("y_w_2").as_double(), this->get_parameter("y_w_3").as_double(), this->get_parameter("y_w_4").as_double()};
      double r_w[4] = {this->get_parameter("r_w_1").as_double(), this->get_parameter("r_w_2").as_double(), this->get_parameter("r_w_3").as_double(), this->get_parameter("r_w_4").as_double()};

      //robot states in robot frame
      double v_x;
      double v_y;
      double omega;
      double W[4];
      double q[8] = {msg->velocity[joint_index_["FL_wheel"]], 
        msg->velocity[joint_index_["BL_wheel"]], 
        msg->velocity[joint_index_["BR_wheel"]], 
        msg->velocity[joint_index_["FR_wheel"]],
        msg->position[joint_index_["FL_steering"]],
        msg->position[joint_index_["BL_steering"]],
        msg->position[joint_index_["BR_steering"]],
        msg->position[joint_index_["FR_steering"]]
      };

      //calculate speed in robot frame
      for(int i =0;i<4;i++){
                v_x = v_x + (cos(q[i+4])/4)*(q[i]*r_w[i]);
                v_y = v_y + (sin(q[i+4])/4)*(q[i]*r_w[i]);
                W[i] = (-y_w[i]*cos(q[i+4]) + x_w[i]*sin(q[i+4]))/(4*pow(x_w[i], 2) + 4*pow(y_w[i], 2));
                omega = omega + W[i]*q[i]*r_w[i];
	    }

      double v_x_global = v_x * cos(theta_) - v_y * sin(theta_);
      double v_y_global = v_x * sin(theta_) + v_y * cos(theta_);

      // Euler integration
      x_     += v_x_global * dt;
      y_     += v_y_global * dt;
      theta_ += omega * dt;

      publishOdometry(x_, y_, theta_, v_x, v_y, omega, current_time);
    }

    void publishOdometry(double x, double y, double theta, double v_x, double v_y, double omega, rclcpp::Time timestamp){
      auto odom_msg = nav_msgs::msg::Odometry();

      odom_msg.header.stamp = this->get_clock()->now();
      odom_msg.header.frame_id = "odom";
      odom_msg.child_frame_id = "robot_base";

      odom_msg.pose.pose.position.x = x;
      odom_msg.pose.pose.position.y = y;
      odom_msg.pose.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, theta);
      odom_msg.pose.pose.orientation.x = q.x();
      odom_msg.pose.pose.orientation.y = q.y();
      odom_msg.pose.pose.orientation.z = q.z();
      odom_msg.pose.pose.orientation.w = q.w();

      // velocities (robot frame)
      odom_msg.twist.twist.linear.x = v_x;
      odom_msg.twist.twist.linear.y = v_y;
      odom_msg.twist.twist.angular.z = omega;

      publisher_odom_->publish(odom_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_states_;
    std::unordered_map<std::string, size_t> joint_index_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr odometry_reset_;
    
    //robot states in global frame
    double x_;
    double y_;
    double theta_;
    rclcpp::Time last_time_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryPublisher>());
  rclcpp::shutdown();
  return 0;
}