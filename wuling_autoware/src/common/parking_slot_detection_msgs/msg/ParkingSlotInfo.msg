# ParkingSlotInfo.msg
std_msgs/Header header  # 用来保存时间戳
float32[] heading_angle   # 保存多个车位的方向角度
geometry_msgs/Point[] center  # 保存多个车位中心点

