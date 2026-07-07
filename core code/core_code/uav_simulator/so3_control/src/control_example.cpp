#include <Eigen/Eigen>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>

namespace
{
constexpr int kCommandQueueSize = 10;
constexpr int kExampleIterations = 500;
constexpr double kStartupDelay = 2.0;
constexpr double kCommandPeriod = 0.01;
} // namespace

int main(int argc, char **argv)
{

  ros::init(argc, argv, "quad_sim_example");
  ros::NodeHandle nh("~");

  ros::Publisher cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", kCommandQueueSize);

  ros::Duration(kStartupDelay).sleep();

  while (ros::ok())
  {

    /*** example 1: position control ***/
    std::cout << "\033[42m"
              << "Position Control to (2,0,1) meters"
              << "\033[0m" << std::endl;
    for (int i = 0; i < kExampleIterations; i++)
    {
      quadrotor_msgs::PositionCommand cmd;
      cmd.position.x = 2.0;
      cmd.position.y = 0.0;
      cmd.position.z = 1.0;
      cmd_pub.publish(cmd);

      ros::Duration(kCommandPeriod).sleep();
      ros::spinOnce();
    }

    /*** example 1: position control ***/
    std::cout << "\033[42m"
              << "Velocity Control to (-1,0,0) meters/second"
              << "\033[0m" << std::endl;
    for (int i = 0; i < kExampleIterations; i++)
    {
      quadrotor_msgs::PositionCommand cmd;
      cmd.position.x = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.position.y = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.position.z = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.velocity.x = -1.0;
      cmd.velocity.y = 0.0;
      cmd.velocity.z = 0.0;
      cmd_pub.publish(cmd);

      ros::Duration(kCommandPeriod).sleep();
      ros::spinOnce();
    }

    /*** example 1: accelleration control ***/
    std::cout << "\033[42m"
              << "Accelleration Control to (1,0,0) meters/second^2"
              << "\033[0m" << std::endl;
    for (int i = 0; i < kExampleIterations; i++)
    {
      quadrotor_msgs::PositionCommand cmd;
      cmd.position.x = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.position.y = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.position.z = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.velocity.x = std::numeric_limits<float>::quiet_NaN();
      cmd.velocity.y = std::numeric_limits<float>::quiet_NaN();
      cmd.velocity.z = std::numeric_limits<float>::quiet_NaN();
      cmd.acceleration.x = 1.0;
      cmd.acceleration.y = 0.0;
      cmd.acceleration.z = 0.0;
      cmd_pub.publish(cmd);

      ros::Duration(kCommandPeriod).sleep();
      ros::spinOnce();
    }

  }

  return 0;
}
