

# 车辆状态 CAN报文对应的部分
uint8 veh_gear_act
float64 veh_spd
float64 veh_acc_exe
float64 veh_brk_exe
float64 veh_str_act

# 档位值定义
uint8 VEH_GEAR_P = 1
uint8 VEH_GEAR_R = 2
uint8 VEH_GEAR_N = 3
uint8 VEH_GEAR_D = 4
uint8 VEH_GEAR_L = 5

# 轮速 CAN报文对应的部分
uint8  whl_pls_rr_valid
uint8  whl_pls_rr_pls_dir
uint16 whl_pls_rr_pls
uint8  whl_pls_rl_pls_valid
uint8  whl_pls_rl_pls_dir
uint16 whl_pls_rl_pls

# 轮速valid值定义
uint8  WHL_PLS_VALID   = 0
uint8  WHL_PLS_INVALID = 1

# 轮速方向值定义
uint8  WHL_PLS_FORWARD  = 1
uint8  WHL_PLS_BACKWARD = 2

# APA 状态 CAN报文对应的部分
uint8 apa_sta_sta
uint8 apa_sta_cmd_echo
uint8 apa_sta_fault_sta
uint8 apa_sta_collision_stat
uint8 apa_sta_scan_state

# APA 状态机值定义
uint8 APA_STA_NO_ACTION_STATE   = 0
uint8 APA_STA_DRIVE_IN_STATE    = 1
uint8 APA_STA_PARK_IN_STATE     = 2
uint8 APA_STA_PARK_OK_STATE     = 3
uint8 APA_STA_PARK_OUT_STATE    = 4
uint8 APA_STA_PARK_OUT_OK_STATE  = 5
uint8 APA_STA_DRIVE_OUT_STATE   = 6
uint8 APA_STA_DRIVE_OUT_OK_STATE = 7
uint8 APA_STA_STATE_ERROR      = 255

# APA 命令值定义
uint8 APA_CMD_NO_PARK_CMD  = 0
uint8 APA_CMD_DRIVE_IN    = 1
uint8 APA_CMD_PARK_IN     = 2
uint8 APA_CMD_PARK_OK     = 3
uint8 APA_CMD_PARK_OUT    = 4
uint8 APA_CMD_DRIVE_OUT   = 5
uint8 APA_CMD_DRIVE_OUT_OK = 6
uint8 APA_CMD_CANCEL     = 128
uint8 APA_CMD_CMD_ERROR   = 255

# APA fault值定义
uint8 APA_NO_FAULT = 0
uint8 APA_FAULT   = 1

# APA Collision值定义
uint8 NO_COLLISION = 0
uint8 COLLISION   = 1

# 扫描车位状态值定义
uint8 NO_SCAN   = 0
uint8 SCANNING = 1



