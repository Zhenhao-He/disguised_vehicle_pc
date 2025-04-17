#ifndef _USBCAN_DRIVER_H_
#define _USBCAN_DRIVER_H_

#include <unistd.h>
#include <stdio.h>

#include <netinet/in.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <signal.h>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"

#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"

#include <zlg_msgs/msg/veh_response.hpp>
#include <zlg_msgs/msg/ctrl_cmd.hpp>
#include <zlg_msgs/msg/frames.hpp>
#include <zlg_msgs/msg/can_frame.hpp>

#include <zlgcan/canframe.h>
#include <zlgcan/config.h>
#include <zlgcan/typedef.h>
#include <zlgcan/zlgcan.h>
#include <cmath>
//zlg驱动头文件


namespace usb_can_ns
{
  using autoware_control_msgs::msg::Control;
  using autoware_vehicle_msgs::msg::GearCommand;

  using autoware_vehicle_msgs::msg::GearReport;
  using autoware_vehicle_msgs::msg::SteeringReport;
  using autoware_vehicle_msgs::msg::VelocityReport;
  using geometry_msgs::msg::AccelWithCovarianceStamped;
  #define MOTOROLA 1
  #define INTEL 0

  typedef unsigned char uint8;
  typedef unsigned short uint16;
  typedef unsigned int uint32;
  typedef unsigned long long uint64;

  typedef char int8;
  typedef short int16;
  typedef int int32;
  typedef long long int64;

  typedef struct
  {
    uint64 BYTE0 : 8;
    uint64 BYTE1 : 8;
    uint64 BYTE2 : 8;
    uint64 BYTE3 : 8;

    uint64 BYTE4 : 8;
    uint64 BYTE5 : 8;
    uint64 BYTE6 : 8;
    uint64 BYTE7 : 8;
  } _BYTES_DATA;

  typedef union {
    uint64 all;
    _BYTES_DATA bit;
  } _UINT64_DATA;

  typedef struct
  {
    rclcpp::Time msgsStamp;
    //   CMSG msgs[CAN_BUS_NUMBER];
  } canMsgs_t;

  class usbcan_driver: public rclcpp::Node
  {
  public:
    usbcan_driver(const rclcpp::NodeOptions & options);
    ~usbcan_driver();

    bool init_usbcan();
    void canFrameInit();
    void acu_CANtransmit();
    void canReceiveAndSend();
    void ReceiveCanCh_0();

    void on_ackermann_cmd(const Control::ConstSharedPtr msg);
    void on_gear_cmd(const GearCommand::ConstSharedPtr msg);
    
    void PrintCanMsgData(ZCAN_Transmit_Data &msg);
    bool Can_Transmit(ZCAN_Transmit_Data *pCAN_obj, uint8 frameNum,uint8 channel);
    void fillFrameData(uint8 frame_data[], uint16 bit_start, uint16 bit_len, bool isMotorola, const uint32 value);
    void fillFrameData_sign(uint8 frame_data[], uint16 bit_start, uint16 bit_len, bool isMotorola, int32_t value);
    uint64 getFrameData(const uint8 frame_data[], uint16 bit_start, uint16 bit_len, bool isMotorola);

    //【指令】加速度、方向盘转角
    rclcpp::Subscription<Control>::SharedPtr sub_ackermann_cmd_;
    //【指令】档位
    rclcpp::Subscription<GearCommand>::SharedPtr sub_gear_cmd_;
    //【状态】方向盘角度
    rclcpp::Publisher<SteeringReport>::SharedPtr pub_steer_report_;
    //【状态】档位
    rclcpp::Publisher<GearReport>::SharedPtr pub_gear_report_;
    //【状态】加速度
    rclcpp::Publisher<AccelWithCovarianceStamped>::SharedPtr pub_acc_report_;
    //【状态】速度
    rclcpp::Publisher<VelocityReport>::SharedPtr pub_velocity_report_;
    //【状态】驱动轮，从动轮速度
    rclcpp::Publisher<VelocityReport>::SharedPtr pub_driven_left_velocity_;
    rclcpp::Publisher<VelocityReport>::SharedPtr pub_driven_right_velocity_;
    rclcpp::Publisher<VelocityReport>::SharedPtr pub_no_driven_left_velocity_;
    rclcpp::Publisher<VelocityReport>::SharedPtr pub_no_driven_right_velocity_;
    //【状态】驱动轮，从动轮脉冲
    rclcpp::Publisher<VelocityReport>::SharedPtr pub_no_driven_left_pulse_;
    rclcpp::Publisher<VelocityReport>::SharedPtr pub_no_driven_right_pulse_;
    rclcpp::Publisher<VelocityReport>::SharedPtr pub_driven_left_pulse_;
    rclcpp::Publisher<VelocityReport>::SharedPtr pub_driven_right_pulse_;
  private:
    
    ZCAN_Transmit_Data  contrlStatusCAN, strAngleCAN, gear_vel_acc_CAN, handbrake_CAN;
    
    rclcpp::TimerBase::SharedPtr timer_;
    uint8 gear_report;
    


  };

} // namespace usb_can_ns

#endif
