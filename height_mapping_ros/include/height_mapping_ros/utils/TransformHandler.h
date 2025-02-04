#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class TransformHandler {
public:
  explicit TransformHandler() : nh_(), tf_listener_(tf_buffer_) {}
  explicit TransformHandler(const ros::NodeHandle &nh) : nh_(nh), tf_listener_(tf_buffer_) {}

  /**
   * @brief Attempts to lookup the transform between two frames.
   * @param target_frame The target frame.
   * @param source_frame The source frame.
   * @param transform Output variable for the transform.
   * @param time Time at which to lookup transform.
   * @return True if successful, false otherwise.
   */
  bool lookupTransform(const std::string &target_frame, const std::string &source_frame,
                       geometry_msgs::TransformStamped &transform, const ros::Time &time = ros::Time(0),
                       const ros::Duration &timeout = ros::Duration(0.1)) {

    try {
      transform = tf_buffer_.lookupTransform(target_frame, source_frame, time, timeout);
      return true;
    } catch (tf2::TransformException &ex) {
      ROS_WARN_STREAM("TF lookup failed: " << ex.what());
      return false;
    }
  }

  static geometry_msgs::TransformStamped combineTransforms(const geometry_msgs::TransformStamped &t1_msg,
                                                           const geometry_msgs::TransformStamped &t2_msg) {
    tf2::Transform t1, t2;
    tf2::fromMsg(t1_msg.transform, t1);
    tf2::fromMsg(t2_msg.transform, t2);

    tf2::Transform t_combined = t1 * t2;
    geometry_msgs::TransformStamped t_combined_msg;
    t_combined_msg.transform = tf2::toMsg(t_combined);
    t_combined_msg.header.frame_id = t2_msg.header.frame_id;
    t_combined_msg.header.stamp = t2_msg.header.stamp;
    t_combined_msg.child_frame_id = t1_msg.child_frame_id;
    return t_combined_msg;
  }

  struct FrameID {
    std::string base_link;
    std::string map;
    std::string sensor;
  };
  /**
   * @brief Loads frame identifiers from the parameter server.
   *
   * @param nh The NodeHandle to use for querying parameters.
   * @return A struct containing the frame IDs.
   */
  static FrameID loadFrameIDs(const ros::NodeHandle &nh) {
    FrameID frame;
    frame.base_link = nh.param<std::string>("base_link", "base_link");
    frame.map = nh.param<std::string>("map", "map");
    frame.sensor = nh.param<std::string>("sensor", "");
    return frame;
  }

private:
  ros::NodeHandle nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
