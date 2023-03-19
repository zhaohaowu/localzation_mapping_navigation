# 系统状态消息
# 基础规则：0 - 正常; !=0 - 异常

Header      header

uint8       system_status   # 系统总体状态

uint8       hw_vcu          # 硬件-VCU状态
uint8       hw_gps          # 硬件-GPS
uint8       hw_lidar_f      # 硬件-激光雷达-前
uint8       hw_lidar_b      # 硬件-激光雷达-后

uint8       sw_global_planner   # 软件-全局路径规划
uint8       sw_local_planner    # 软件-局部路径规划