#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

class imu_adapter_node
{
    protected:
  // Subscriber objects
  ros::Subscriber imu_fusion_sub_;

  // Publisher objects
  ros::Publisher imu_fusion_pub_;

  // Message objects
  sensor_msgs::Imu imu_out_;

public:    
    imu_adapter_node()
    {
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_("~");

        // init subscriber
        imu_fusion_sub_ = nh_.subscribe("imu", 1000, &imu_adapter_node::imu_fusion_callback, this);

        // init publisher
        imu_fusion_pub_ = nh_.advertise<sensor_msgs::Imu>("imu_fusion", 10, false);

        // init covariances grabbed from param server
        init_covariances(nh_private_);
    }
    
    void imu_fusion_callback(const sensor_msgs::Imu::ConstPtr& imu_fusion_msg)
    {
    // Populate header
    imu_out_.header.stamp = ros::Time::now();

    // Populate orientation data
    imu_out_.orientation.x = imu_fusion_msg->orientation.x;
    imu_out_.orientation.y = imu_fusion_msg->orientation.y;
    imu_out_.orientation.z = imu_fusion_msg->orientation.z;
    imu_out_.orientation.w = imu_fusion_msg->orientation.w;

    // Populate angular velocity data
    imu_out_.angular_velocity.x = imu_fusion_msg->angular_velocity.x;
    imu_out_.angular_velocity.y = imu_fusion_msg->angular_velocity.y;
    imu_out_.angular_velocity.z = imu_fusion_msg->angular_velocity.z;

    // Populate linear acceleration data
    imu_out_.linear_acceleration.x = imu_fusion_msg->linear_acceleration.x;
    imu_out_.linear_acceleration.y = imu_fusion_msg->linear_acceleration.y;
    imu_out_.linear_acceleration.z = imu_fusion_msg->linear_acceleration.z;

    // Publish the sensor_msgs/Imu message
        imu_fusion_pub_.publish(imu_out_);
    }

    void init_covariances(ros::NodeHandle &nh_private_)
    {
    // Create the vectors to store the covariance matrix arrays
    std::vector<double> orientation_covar;
    std::vector<double> ang_vel_covar;
    std::vector<double> linear_accel_covar;
    std::vector<double> pose_covar;

    // Grab the parameters and populate the vectors
    nh_private_.getParam("imu_orientation_covariance", orientation_covar);
    nh_private_.getParam("imu_angular_velocity_covariance", ang_vel_covar);
    nh_private_.getParam("imu_linear_acceleration_covariance", linear_accel_covar);
    nh_private_.getParam("pose_covariance", pose_covar);

    // Iterate through each vector and populate the respective message fields
    for (int i = 0; i < orientation_covar.size(); i++)
      imu_out_.orientation_covariance[i] = orientation_covar.at(i);

    for (int i = 0; i < ang_vel_covar.size(); i++)
      imu_out_.angular_velocity_covariance[i] = ang_vel_covar.at(i);

    for (int i = 0; i < linear_accel_covar.size(); i++)
      imu_out_.linear_acceleration_covariance[i] = linear_accel_covar.at(i);
    }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_adapter");

  imu_adapter_node imu_adapter;

  ros::spin();

  return 0;
}

