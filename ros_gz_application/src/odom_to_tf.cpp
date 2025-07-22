#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"                        
#include "geometry_msgs/msg/transform_stamped.hpp"          
#include "tf2_ros/transform_broadcaster.h" 
#include "geometry_msgs/msg/pose_stamped.hpp"                 

class OdomToTfBroadcaster : public rclcpp::Node
{
public:
  OdomToTfBroadcaster()
  : Node("odom_to_tf_broadcaster")
  {
    
    tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(this);  

    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/vehicle/odom", 10,
      std::bind(&OdomToTfBroadcaster::odom_callback, this, std::placeholders::_1));

    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/goal_update", 10); 
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    
    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::PoseStamped goal_msg;

    t.header.stamp    = msg->header.stamp;           
    t.header.frame_id = "map";     
    t.child_frame_id  = "odom_follow";                 

    
    t.transform.translation.x = msg->pose.pose.position.x - 3.0; // Décalage de 3.0 en x
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;

   
    t.transform.rotation = msg->pose.pose.orientation;

    // Publication du message de goal
    goal_msg.header.stamp = msg->header.stamp;
    goal_msg.header.frame_id = "map";
    goal_msg.pose.position.x = t.transform.translation.x; // Décalage de 1.0 en x
    goal_msg.pose.position.y = t.transform.translation.y;
    goal_msg.pose.position.z = t.transform.translation.z;
    goal_msg.pose.orientation = t.transform.rotation;

    // Publication sur /tf
    tf_broadcaster_->sendTransform(t);     
    goal_pub_->publish(goal_msg); // Publier le message de goal
    RCLCPP_INFO(this->get_logger(), "Published transform and goal message");         
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
