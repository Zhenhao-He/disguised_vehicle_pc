#include "convert_driver/convert_driver.h"

#include <cstring>
#include <thread>
#include <unistd.h>
#include <algorithm>  // std::swap


using namespace std;

namespace usb_can_ns
{
    
DEVICE_HANDLE hDevice;
CHANNEL_HANDLE hChannels[8];
ZCAN_CHANNEL_INIT_CONFIG config;
ZCAN_DEVICE_INFO info;
usbcan_driver::usbcan_driver(const rclcpp::NodeOptions & options): Node("usbcan_driver", options)
{
    canFrameInit();  
    bool initResult = false;
    initResult = init_usbcan();
    if (!initResult)
    {
        ZCAN_CloseDevice(hDevice); //ZLG关闭设备
        sleep(1);
    }
    using rclcpp::QoS;
    using std::placeholders::_1;
    using std::placeholders::_2;
    sub_ackermann_cmd_ = create_subscription<Control>(
    "~/input/command/control_cmd", QoS{1},std::bind(&usbcan_driver::on_ackermann_cmd, this, _1));

    sub_gear_cmd_ = create_subscription<GearCommand>(
    "~/input/command/gear_cmd", QoS{1},std::bind(&usbcan_driver::on_gear_cmd, this, _1));
    
    pub_steer_report_ = create_publisher<SteeringReport>("~/output/steer_state", QoS{1});
    pub_gear_report_ = create_publisher<GearReport>("~/output/gear_state", QoS{1});
    pub_acc_report_ = create_publisher<AccelWithCovarianceStamped>("~/output/acceleration_state", QoS{1});
    pub_velocity_report_ = create_publisher<VelocityReport>("~/output/velocity_state", QoS{1});

    pub_driven_left_velocity_ = create_publisher<VelocityReport>("~/output/driven_left_velocity", QoS{1});
    pub_driven_right_velocity_ = create_publisher<VelocityReport>("~/output/driven_right_velocity", QoS{1});
    pub_no_driven_left_velocity_ = create_publisher<VelocityReport>("~/output/no_driven_left_velocity", QoS{1});
    pub_no_driven_right_velocity_ = create_publisher<VelocityReport>("~/output/no_driven_right_velocity", QoS{1});

    pub_driven_left_pulse_ = create_publisher<VelocityReport>("~/output/driven_left_pulse", QoS{1});
    pub_driven_right_pulse_ = create_publisher<VelocityReport>("~/output/driven_right_pulse", QoS{1});
    pub_no_driven_left_pulse_ = create_publisher<VelocityReport>("~/output/no_driven_left_pulse", QoS{1});
    pub_no_driven_right_pulse_ = create_publisher<VelocityReport>("~/output/no_driven_right_pulse", QoS{1});

    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&usbcan_driver::canReceiveAndSend, this));
    std::cout << "construction is done" << std::endl;

}

usbcan_driver::~usbcan_driver(void)
{
    std::cout << "close" << std::endl;

    ZCAN_CloseDevice(hDevice); //ZLG关闭设备
}

void usbcan_driver::canFrameInit()
{

    unsigned char data1D3[] = {0x00, 0x00, 0x40, 0x00, 0x01, 0x0A, 0x00, 0x00};//16个计数器，标志位
    contrlStatusCAN.frame.can_id = CAN_ID(0x1D3,0,0,0);
    contrlStatusCAN.transmit_type = 0;
    contrlStatusCAN.frame.can_dlc = 8;
    memcpy(contrlStatusCAN.frame.data, data1D3, 8);
    
    unsigned char data1D5[] = {0xA0, 0x00, 0x73, 0xA0, 0x00, 0x00, 0x00, 0x00}; //4个计数器，转角
    strAngleCAN.frame.can_id = CAN_ID(0x1D5,0,0,0);
    strAngleCAN.transmit_type = 0;
    strAngleCAN.frame.can_dlc = 8;
    memcpy(strAngleCAN.frame.data, data1D5, 8);
    
    unsigned char data1D2[] = {0xB8, 0x00, 0xA1, 0x00, 0x00, 0x00, 0x00, 0x00}; //16个计数器，加速度
    gear_vel_acc_CAN.frame.can_id = CAN_ID(0x1D2,0,0,0);
    gear_vel_acc_CAN.transmit_type = 0;
    gear_vel_acc_CAN.frame.can_dlc = 8;
    memcpy(gear_vel_acc_CAN.frame.data, data1D2, 8);
    
    unsigned char data1D1[] = {0x60, 0x80, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00}; //4个计数器,手刹
    handbrake_CAN.frame.can_id = CAN_ID(0x1D1,0,0,0);
    handbrake_CAN.transmit_type = 0;
    handbrake_CAN.frame.can_dlc = 8;
    memcpy(handbrake_CAN.frame.data, data1D1, 8);

}

bool usbcan_driver::init_usbcan()
{
    hDevice = ZCAN_OpenDevice(ZCAN_USBCAN_8E_U, 0x00, 0);
    if (hDevice == INVALID_DEVICE_HANDLE )//打开ZLG设备
    {

        std::cout <<"open deivce error\n" << std::endl;
        return false;
    }
    if (ZCAN_GetDeviceInf(hDevice, &info) < 0)//从zlgcan_node中添加
    {
        printf("ZCAN_GetDeviceInf() failed!\n");
        ZCAN_CloseDevice(hDevice);
        return false;
    }
    else
    {
        printf("ZCAN_GetDeviceInf() Version:(%d:%d),Serial:%s.\n", info.hw_Version,
                info.fw_Version, info.str_Serial_Num);
    }
    //ZLG config设置初始化
    config.can_type = 0;
    config.can.acc_code = 0x00;
    config.can.acc_mask = 0xffffffff;
    config.can.filter = 0x01;
    config.can.timing0 = 0x00;
    config.can.timing1 = 0x1C;
    config.can.mode = 0;
            
    //通道初始化
    int i;
    for (i = 0; i < 8; i++)
    {
        hChannels[i] = ZCAN_InitCAN(hDevice, i, &config);
        if (hChannels[i] == INVALID_CHANNEL_HANDLE)
        {
            printf("ZCAN_InitCAN() failed at channel <%d>.\n", i);
            ZCAN_CloseDevice(hDevice);
            return 0;
        }
        //Start CAN channels
        int result = ZCAN_ResetCAN(hChannels[i]);
        if (result == -1)
        {
            printf("ZCAN_ResetCAN() failed!, channel=%d!\n", i);
        }
        usleep(100);

        result = ZCAN_ClearBuffer(hChannels[i]);
        if (result == -1)
        {
            printf("ZCAN_ClearBuffer() failed!, channel=%d!\n", i);
        }

        result = ZCAN_StartCAN(hChannels[i]);
        if (result < 0)
        {

            printf("ZCAN_Start() failed, channel=%d!\n", i);
            ZCAN_CloseDevice(hDevice);

            return 0;
        }

    }

    printf("ZCAN_Start() success for all channels!\n");
    
    return true;
}
void usbcan_driver::canReceiveAndSend()
{

        
    acu_CANtransmit();//控制底盘

    ReceiveCanCh_0() ;//从底盘读取

}

void usbcan_driver::acu_CANtransmit()
{

    /*
    //-----------------1D1-----------------//注意：1D1和1D3要组合起来发，手刹才有效
    static uint8_t handbrake_cnt = 0;
    handbrake_cnt++;
    if (handbrake_cnt == 4)
    {
        handbrake_cnt = 0;
    }
    //添加计数器
    fillFrameData(handbrake_CAN.frame.data, 48, 2, MOTOROLA, handbrake_cnt); 

    // 计算校验和
    uint8_t handbrake_checksum = 0;
    for (int i = 0; i < 7; ++i) {
        handbrake_checksum += handbrake_CAN.frame.data[i];
    }
    handbrake_checksum &= 0xFF; // 取最低字节
    handbrake_CAN.frame.data[7] = handbrake_checksum;

    //PrintCanMsgData(handbrake_CAN);
    Can_Transmit(&handbrake_CAN, 1,0);//通道0   
    */


   static uint8_t contrlStatus_cnt = 0;

   static uint8_t contrlStatus_cnt_flag = 0;
   contrlStatus_cnt_flag++;
   if(contrlStatus_cnt_flag==3)
   {
    contrlStatus_cnt_flag=0;

    //-----------------1D3-----------------//
    contrlStatus_cnt++;
    if (contrlStatus_cnt == 16)
    {
        contrlStatus_cnt = 0;
    }
    //添加计数器
    fillFrameData(contrlStatusCAN.frame.data, 48, 4, MOTOROLA, contrlStatus_cnt); 

    // 计算校验和
    uint8_t contrlStatus_checksum = 0;
    for (int i = 0; i < 7; ++i) {
        contrlStatus_checksum += contrlStatusCAN.frame.data[i];
    }
    contrlStatus_checksum &= 0xFF; // 取最低字节
    contrlStatusCAN.frame.data[7] = contrlStatus_checksum;

    //PrintCanMsgData(contrlStatusCAN);
    Can_Transmit(&contrlStatusCAN, 1,0);//通道0 
   }
    
    
    //-----------------1D5-----------------//注意：1D5和1D3要组合起来发，转角才有效
    static uint8_t strAngle_cnt = 0;

    strAngle_cnt++;
    if (strAngle_cnt == 4)
    {
        strAngle_cnt = 0;
    }
    //添加计数器
    fillFrameData(strAngleCAN.frame.data, 48, 2, MOTOROLA, strAngle_cnt); 

    // 计算校验和
    uint8_t strAngle_checksum = 0;
    for (int i = 0; i < 7; ++i) {
        strAngle_checksum += strAngleCAN.frame.data[i];
    }
    strAngle_checksum &= 0xFF; // 取最低字节
    strAngleCAN.frame.data[7] = strAngle_checksum;
    //PrintCanMsgData(strAngleCAN);
    Can_Transmit(&strAngleCAN, 1,0);//通道0

    //-----------------1D2-----------------//

    static uint8_t gear_vel_acc_cnt = 0;
    gear_vel_acc_cnt++;
    if (gear_vel_acc_cnt == 16)
    {
        gear_vel_acc_cnt = 0;
    }
    //添加计数器
    fillFrameData(gear_vel_acc_CAN.frame.data, 48, 4, MOTOROLA, gear_vel_acc_cnt); 

    // 计算校验和
    uint8_t gear_vel_acc_checksum = 0;
    for (int i = 0; i < 7; ++i) {
        gear_vel_acc_checksum += gear_vel_acc_CAN.frame.data[i];
    }
    gear_vel_acc_checksum &= 0xFF; // 取最低字节
    gear_vel_acc_CAN.frame.data[7] = gear_vel_acc_checksum;

    //PrintCanMsgData(gear_vel_acc_CAN);
    Can_Transmit(&gear_vel_acc_CAN, 1,4);//通道4
    
}
//接收底盘传回的转角和档位信号
void usbcan_driver::ReceiveCanCh_0() 
{
    int reclen = 0;
    ZCAN_Receive_Data rec[2500];
    static float previous_speed = 0.0;         // 上一时刻的速度
    static rclcpp::Time previous_time= get_clock()->now();          // 上一时刻的时间
    
    if ((reclen = ZCAN_Receive(hChannels[0], rec, 2500, 0)) > 0) //CAN 2接收数据
    {
        for (size_t i = 0; i < reclen; i++)
        {   SteeringReport Steer_msg_report;
            GearReport Gear_msg_report;
            VelocityReport Velocity_msg_report;

            
            VelocityReport driven_right_Velocity_msg;
            VelocityReport driven_left_Velocity_msg;
            VelocityReport no_driven_right_Velocity_msg;
            VelocityReport no_driven_left_Velocity_msg;

            VelocityReport driven_right_pulse_msg;
            VelocityReport driven_left_pulse_msg;
            VelocityReport no_driven_right_pulse_msg;
            VelocityReport no_driven_left_pulse_msg;

            rclcpp::Time current_time;
            float delta_time;
            float acceleration;
            AccelWithCovarianceStamped acc_msg;

            switch (rec[i].frame.can_id) 
            {
                case 0x1E5://方向盘转角
                    double current_steer;
                    current_steer=getFrameData(rec[i].frame.data, 16, 16, MOTOROLA)*0.0625;
                    if(current_steer>740)
                    {
                        current_steer=current_steer-4096;
                    }
                    current_steer=current_steer/ (-180.0 / M_PI)/17; //最大+/- 32度 待改
                    //记得把方向盘转化成前轮转角
                    Steer_msg_report.stamp = get_clock()->now();
                    Steer_msg_report.steering_tire_angle=current_steer;
                    pub_steer_report_->publish(Steer_msg_report);
                    //RCLCPP_INFO(rclcpp::get_logger("0x1E5 Steer_msg_report.steering_tire_angle"), " value=%f",current_steer);
                    break;
                case 0x155://档位
                    uint16 current_gear_value;
                    current_gear_value=getFrameData(rec[i].frame.data, 33, 3, MOTOROLA);//改，测试是否正确
                    //RCLCPP_INFO(rclcpp::get_logger("0x155 current_gear_value"), "=%X",current_gear_value);
                    
                    if(current_gear_value==0)//N档
                    {
                        gear_report=1;
                    }
                    else if(current_gear_value==1)//D档
                    {
                        gear_report=2;
                    }
                    else if(current_gear_value==2)//R档
                    {
                        gear_report=20;
                    }
                    else if(current_gear_value==3)//P档
                    {
                        gear_report=22;
                    }
                    else
                    {
                        return;
                    }
                    Gear_msg_report.stamp = get_clock()->now();
                    Gear_msg_report.report =gear_report     ;//从线控底盘读取
                    pub_gear_report_->publish(Gear_msg_report);
                    break;
                case 0x348://左右驱动轮 速度
                    double driven_speed_left;
                    double driven_speed_right;
                    double current_speed;
                    driven_speed_left=getFrameData(rec[i].frame.data, 23, 16, MOTOROLA)*0.01;//左驱动轮速度
                    driven_speed_right=getFrameData(rec[i].frame.data, 38, 16, MOTOROLA)*0.01;//右驱动轮速度
                    current_speed=(driven_speed_left+driven_speed_right)/(2*3.6); //换算成m/s

                    if(gear_report==20){current_speed=current_speed*-1;}//若为R挡乘以-1
                    
                    //############获取加速度############
                    current_time = get_clock()->now();
                    delta_time = (current_time - previous_time).seconds();
                    if (delta_time > 0.02) // 只在时间差大于某个阈值时才计算加速度 
                    {    
                        acceleration = (current_speed - previous_speed) / delta_time;
                        //RCLCPP_INFO(rclcpp::get_logger("acceleration_time"), "previous time: %f seconds", previous_time.seconds()); // 打印当前时间（秒）
                        //RCLCPP_INFO(rclcpp::get_logger("acceleration_time"), "Current time: %f seconds", current_time.seconds()); // 打印当前时间（秒）

                        previous_speed = current_speed;
                        previous_time = current_time;

                        acc_msg.header.frame_id = "/base_link";
                        acc_msg.header.stamp = get_clock()->now();
                        acc_msg.accel.accel.linear.x = acceleration;
                        acc_msg.accel.accel.linear.y = 0;
                        pub_acc_report_->publish(acc_msg);
                        //RCLCPP_INFO(rclcpp::get_logger("acceleration:"), " value=%f",acceleration);
                    }
                    else {
                        //RCLCPP_WARN(this->get_logger(), "Time difference too small, skipping acceleration calculation");
                    }
                    //############获取加速度############

                    Velocity_msg_report.header.stamp = get_clock()->now();
                    Velocity_msg_report.longitudinal_velocity=current_speed;
                    pub_velocity_report_->publish(Velocity_msg_report);

                    driven_left_Velocity_msg.header.stamp = get_clock()->now();
                    driven_left_Velocity_msg.longitudinal_velocity=driven_speed_left/3.6;
                    pub_driven_left_velocity_->publish(driven_left_Velocity_msg);
                    //RCLCPP_INFO(rclcpp::get_logger("driven_speed_left:"), " value=%f",driven_speed_left/3.6);
                     
                    driven_right_Velocity_msg.header.stamp = get_clock()->now();
                    driven_right_Velocity_msg.longitudinal_velocity=driven_speed_right/3.6;
                    pub_driven_right_velocity_->publish(driven_right_Velocity_msg);
                    //RCLCPP_INFO(rclcpp::get_logger("driven_speed_right:"), " value=%f",driven_speed_right/3.6);        
                case 0x34A://左右从动轮 速度
                    double no_driven_speed_left;
                    double no_driven_speed_right;

                    no_driven_speed_left=getFrameData(rec[i].frame.data, 23, 16, MOTOROLA)*0.01;//左从动轮速度
                    no_driven_speed_right=getFrameData(rec[i].frame.data, 38, 16, MOTOROLA)*0.01;//右从动轮速度

                    no_driven_left_Velocity_msg.header.stamp = get_clock()->now();
                    no_driven_left_Velocity_msg.longitudinal_velocity=no_driven_speed_left/3.6;
                    pub_no_driven_left_velocity_->publish(no_driven_left_Velocity_msg);
                    //RCLCPP_INFO(rclcpp::get_logger("no_driven_speed_left:"), " value=%f",no_driven_speed_left/3.6);
                     
                    no_driven_right_Velocity_msg.header.stamp = get_clock()->now();
                    no_driven_right_Velocity_msg.longitudinal_velocity=no_driven_speed_right/3.6;
                    pub_no_driven_right_velocity_->publish(no_driven_right_Velocity_msg);
                    //RCLCPP_INFO(rclcpp::get_logger("no_driven_speed_right:"), " value=%f",no_driven_speed_right/3.6); 


                case 0x0C5://左右从动轮脉冲
                    double no_driven_pulse_left;
                    double no_driven_pulse_right;

                    no_driven_pulse_left=getFrameData(rec[i].frame.data, 8, 10, MOTOROLA);//左从动轮脉冲

                    if(getFrameData(rec[i].frame.data, 3, 3, MOTOROLA)==0) no_driven_pulse_left=9999;//Backward就乘以-1
                    else if(getFrameData(rec[i].frame.data, 3, 3, MOTOROLA)==1) ;
                    else if(getFrameData(rec[i].frame.data, 3, 3, MOTOROLA)==2) no_driven_pulse_left=no_driven_pulse_left*-1;
                    else if(getFrameData(rec[i].frame.data, 3, 3, MOTOROLA)==3) no_driven_pulse_left=0;
                    else if(getFrameData(rec[i].frame.data, 3, 3, MOTOROLA)==4) no_driven_pulse_left=-8888;
                    else no_driven_pulse_left=0;
                    

                    no_driven_pulse_right=getFrameData(rec[i].frame.data, 40, 10, MOTOROLA);//右从动轮脉冲
                    if(getFrameData(rec[i].frame.data, 35, 3, MOTOROLA)==0) no_driven_pulse_right=9999;//Backward就乘以-1
                    else if(getFrameData(rec[i].frame.data, 35, 3, MOTOROLA)==1) ;
                    else if(getFrameData(rec[i].frame.data, 35, 3, MOTOROLA)==2) no_driven_pulse_right=no_driven_pulse_right*-1;
                    else if(getFrameData(rec[i].frame.data, 35, 3, MOTOROLA)==3) no_driven_pulse_right=0;
                    else if(getFrameData(rec[i].frame.data, 35, 3, MOTOROLA)==4) no_driven_pulse_right=-8888;
                    else no_driven_pulse_right=0;


                    no_driven_left_pulse_msg.header.stamp = get_clock()->now();
                    no_driven_left_pulse_msg.longitudinal_velocity=no_driven_pulse_left;
                    pub_no_driven_left_pulse_->publish(no_driven_left_pulse_msg);
                    //RCLCPP_INFO(rclcpp::get_logger("no_driven_pulse_left:"), " value=%f",no_driven_pulse_left);
                     
                    no_driven_right_pulse_msg.header.stamp = get_clock()->now();
                    no_driven_right_pulse_msg.longitudinal_velocity=no_driven_pulse_right;
                    pub_no_driven_right_pulse_->publish(no_driven_right_pulse_msg);
                    //RCLCPP_INFO(rclcpp::get_logger("no_driven_pulse_right:"), " value=%f",no_driven_pulse_right);   
                case 0x0C1://左右驱动轮脉冲
                    double driven_pulse_left;
                    double driven_pulse_right;

                    driven_pulse_left=getFrameData(rec[i].frame.data, 8, 10, MOTOROLA);//左从动轮脉冲
                    if(getFrameData(rec[i].frame.data, 3, 3, MOTOROLA)==0) driven_pulse_left=9999;//Backward就乘以-1
                    else if(getFrameData(rec[i].frame.data, 3, 3, MOTOROLA)==1) ;
                    else if(getFrameData(rec[i].frame.data, 3, 3, MOTOROLA)==2) driven_pulse_left=driven_pulse_left*-1;
                    else if(getFrameData(rec[i].frame.data, 3, 3, MOTOROLA)==3) driven_pulse_left=0;
                    else if(getFrameData(rec[i].frame.data, 3, 3, MOTOROLA)==4) driven_pulse_left=-8888;
                    else driven_pulse_left=0;

                    driven_pulse_right=getFrameData(rec[i].frame.data, 40, 10, MOTOROLA);//右从动轮脉冲
                     if(getFrameData(rec[i].frame.data, 35, 3, MOTOROLA)==0) driven_pulse_right=9999;//Backward就乘以-1
                    else if(getFrameData(rec[i].frame.data, 35, 3, MOTOROLA)==1) ;
                    else if(getFrameData(rec[i].frame.data, 35, 3, MOTOROLA)==2) driven_pulse_right=driven_pulse_right*-1;
                    else if(getFrameData(rec[i].frame.data, 35, 3, MOTOROLA)==3) driven_pulse_right=0;
                    else if(getFrameData(rec[i].frame.data, 35, 3, MOTOROLA)==4) driven_pulse_right=-8888;
                    else driven_pulse_right=0;
                    
                    driven_left_pulse_msg.header.stamp = get_clock()->now();
                    driven_left_pulse_msg.longitudinal_velocity=driven_pulse_left;
                    pub_driven_left_pulse_->publish(driven_left_pulse_msg);
                    //RCLCPP_INFO(rclcpp::get_logger("driven_pulse_left:"), " value=%f",driven_pulse_left);
                     
                    driven_right_pulse_msg.header.stamp = get_clock()->now();
                    driven_right_pulse_msg.longitudinal_velocity=driven_pulse_right;
                    pub_driven_right_pulse_->publish(driven_right_pulse_msg);
                    //RCLCPP_INFO(rclcpp::get_logger("driven_pulse_right:"), " value=%f",driven_pulse_right);
                default:
                    break;
            }
        }
        
    }
}

//###########################以下属于回调函数###########################
//设置转角+加速度
void usbcan_driver::on_ackermann_cmd(const Control::ConstSharedPtr msg) 
{ 
    double steer = msg->lateral.steering_tire_angle;
    //记得把前轮转角转化成方向盘转角
    steer=steer* (-180.0 / M_PI)*17; 
    //RCLCPP_INFO(rclcpp::get_logger("msg->lateral.steering_tire_angle "), " value=%f",steer);
    steer=(steer+740)*10;
    fillFrameData(strAngleCAN.frame.data, 26, 14, MOTOROLA, steer); 
    
    
    double accel = msg->longitudinal.acceleration;
    if(accel<-0.3)
    {
        accel=-0.3;
    }
    if(accel>0.3)
    {
        accel=0.3;
    }
    accel=accel/0.01;
    //RCLCPP_INFO(rclcpp::get_logger("msg->lateral.acceleration "), " value=%f",accel);
    fillFrameData_sign(gear_vel_acc_CAN.frame.data, 8, 11, MOTOROLA, accel); //传入有符号的填充函数
}

//设置档位
void usbcan_driver::on_gear_cmd(const GearCommand::ConstSharedPtr msg) 
{ 

    const auto gear_cmd = msg->command;
    uint8 gear_value;
    if(gear_cmd==0||gear_cmd==1)//N档
    {
        gear_value=0;
    }
    else if((gear_cmd>=2 && gear_cmd<=19)||(gear_cmd>=23 && gear_cmd<24))//D档
    {
        gear_value=1;
    }
    else if(gear_cmd==20||gear_cmd==21)//R档
    {
        gear_value=2;
    }
    else if(gear_cmd==22)//P档
    {
        gear_value=3;
    }
    else
    {
        return;
    }

    fillFrameData(gear_vel_acc_CAN.frame.data, 21, 2, MOTOROLA, gear_value); //改，测试是否正确
    
} 
 

//###########################以下属于库函数###########################
void usbcan_driver::PrintCanMsgData(ZCAN_Transmit_Data &msg)
{
    RCLCPP_INFO(rclcpp::get_logger("ZCAN_Transmit_Data"), "ID=%.2X : DATA=[%.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X]",
    msg.frame.can_id,
    msg.frame.data[0],
    msg.frame.data[1],
    msg.frame.data[2],
    msg.frame.data[3],
    msg.frame.data[4],
    msg.frame.data[5],
    msg.frame.data[6],
    msg.frame.data[7]);
}
bool usbcan_driver::Can_Transmit(ZCAN_Transmit_Data *pCAN_obj, uint8 frameNum,uint8 channel)
{
    uint32 res;
    res = ZCAN_Transmit(hChannels[channel],  pCAN_obj, 1);//CAN 2发送数据

    if (res <= 0)
    {
        std::cout << "CAN  transmit is fail!" << std::endl;
        if (res = -1)
            std::cout << "USB-CAN doesn't exist or cable is loose!" << std::endl;
        return false;
    }
    else
    {
        //RCLCPP_INFO(rclcpp::get_logger("ZCAN_Transmit_Data ok-------------"), "");
        return true;
    }
    
}

void usbcan_driver::fillFrameData(uint8 frame_data[], uint16 bit_start, uint16 bit_len, bool isMotorola, uint32 value)
{
    uint32 index, mask;
    /*_UINT64_DATA是小端模式*/
    _UINT64_DATA long_data;
    _UINT64_DATA frame_clear;
    uint8 CellinFrontRow, CellBeforeCurRow, ReminderCell;

    if (isMotorola == MOTOROLA)
    {
        index = 56 + (bit_start & 7) - (bit_start & 0xFFF8);
        mask = (0x1ULL << bit_len) - 1;
        long_data.all = value & mask;
        long_data.all <<= index;

        frame_clear.all = mask;
        frame_clear.all <<= index;
        frame_clear.all = ~frame_clear.all;

        frame_data[7] &= frame_clear.bit.BYTE0;
        frame_data[6] &= frame_clear.bit.BYTE1;
        frame_data[5] &= frame_clear.bit.BYTE2;
        frame_data[4] &= frame_clear.bit.BYTE3;
        frame_data[3] &= frame_clear.bit.BYTE4;
        frame_data[2] &= frame_clear.bit.BYTE5;
        frame_data[1] &= frame_clear.bit.BYTE6;
        frame_data[0] &= frame_clear.bit.BYTE7;

        frame_data[7] |= long_data.bit.BYTE0;
        frame_data[6] |= long_data.bit.BYTE1;
        frame_data[5] |= long_data.bit.BYTE2;
        frame_data[4] |= long_data.bit.BYTE3;
        frame_data[3] |= long_data.bit.BYTE4;
        frame_data[2] |= long_data.bit.BYTE5;
        frame_data[1] |= long_data.bit.BYTE6;
        frame_data[0] |= long_data.bit.BYTE7;
    }
    else
    {
        index = bit_start;
        mask = (0x1ULL << bit_len) - 1;
        long_data.all = value & mask;
        long_data.all <<= index;
        frame_data[0] |= long_data.bit.BYTE0;
        frame_data[1] |= long_data.bit.BYTE1;
        frame_data[2] |= long_data.bit.BYTE2;
        frame_data[3] |= long_data.bit.BYTE3;
        frame_data[4] |= long_data.bit.BYTE4;
        frame_data[5] |= long_data.bit.BYTE5;
        frame_data[6] |= long_data.bit.BYTE6;
        frame_data[7] |= long_data.bit.BYTE7;
    }
}

void usbcan_driver::fillFrameData_sign(uint8 frame_data[], uint16 bit_start, uint16 bit_len, bool isMotorola, int32_t value)
{

    if (value < 0) {
        value = -value;
        value = ~value + 1;
    }

    uint32 index, mask;
    /*_UINT64_DATA是小端模式*/
    _UINT64_DATA long_data;
    _UINT64_DATA frame_clear;
    uint8 CellinFrontRow, CellBeforeCurRow, ReminderCell;

    if (isMotorola == MOTOROLA)
    {
        index = 56 + (bit_start & 7) - (bit_start & 0xFFF8);
        mask = (0x1ULL << bit_len) - 1;
        long_data.all = value & mask;
        long_data.all <<= index;

        frame_clear.all = mask;
        frame_clear.all <<= index;
        frame_clear.all = ~frame_clear.all;

        frame_data[7] &= frame_clear.bit.BYTE0;
        frame_data[6] &= frame_clear.bit.BYTE1;
        frame_data[5] &= frame_clear.bit.BYTE2;
        frame_data[4] &= frame_clear.bit.BYTE3;
        frame_data[3] &= frame_clear.bit.BYTE4;
        frame_data[2] &= frame_clear.bit.BYTE5;
        frame_data[1] &= frame_clear.bit.BYTE6;
        frame_data[0] &= frame_clear.bit.BYTE7;

        frame_data[7] |= long_data.bit.BYTE0;
        frame_data[6] |= long_data.bit.BYTE1;
        frame_data[5] |= long_data.bit.BYTE2;
        frame_data[4] |= long_data.bit.BYTE3;
        frame_data[3] |= long_data.bit.BYTE4;
        frame_data[2] |= long_data.bit.BYTE5;
        frame_data[1] |= long_data.bit.BYTE6;
        frame_data[0] |= long_data.bit.BYTE7;
    }
    else
    {
        index = bit_start;
        mask = (0x1ULL << bit_len) - 1;
        long_data.all = value & mask;
        long_data.all <<= index;
        frame_data[0] |= long_data.bit.BYTE0;
        frame_data[1] |= long_data.bit.BYTE1;
        frame_data[2] |= long_data.bit.BYTE2;
        frame_data[3] |= long_data.bit.BYTE3;
        frame_data[4] |= long_data.bit.BYTE4;
        frame_data[5] |= long_data.bit.BYTE5;
        frame_data[6] |= long_data.bit.BYTE6;
        frame_data[7] |= long_data.bit.BYTE7;
    }
}

uint64 usbcan_driver::getFrameData(const uint8 frame_data[], uint16 bit_start, uint16 bit_len, bool isMotorola)
{
    uint32 index;
    uint64 Mask;
    _UINT64_DATA Get_data;
    uint64 RetVal;

    if (isMotorola == MOTOROLA)
    {
        index = 56 + (bit_start & 7) - (bit_start & 0xFFF8);
        Mask = (0x1ULL << bit_len) - 1;

        Get_data.bit.BYTE0 = frame_data[7];
        Get_data.bit.BYTE1 = frame_data[6];
        Get_data.bit.BYTE2 = frame_data[5];
        Get_data.bit.BYTE3 = frame_data[4];
        Get_data.bit.BYTE4 = frame_data[3];
        Get_data.bit.BYTE5 = frame_data[2];
        Get_data.bit.BYTE6 = frame_data[1];
        Get_data.bit.BYTE7 = frame_data[0];

        RetVal = (Get_data.all >> index) & Mask;
    }
    else
    {
        Get_data.bit.BYTE0 = frame_data[0];
        Get_data.bit.BYTE1 = frame_data[1];
        Get_data.bit.BYTE2 = frame_data[2];
        Get_data.bit.BYTE3 = frame_data[3];
        Get_data.bit.BYTE4 = frame_data[4];
        Get_data.bit.BYTE5 = frame_data[5];
        Get_data.bit.BYTE6 = frame_data[6];
        Get_data.bit.BYTE7 = frame_data[7];

        index = bit_start;
        Mask = (0x1ULL << bit_len) - 1;
        RetVal = Get_data.all & Mask;
        RetVal >>= index;
    }

    return RetVal;
}



} // end of namespace 
