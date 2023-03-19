## Time stamped drive command for robots with Ackermann steering.
#  $Id$

Header          header

int16   Vol         # 电池电压
int16   Curr        # 电池电流
int16   Quantity    # 电池电量，单位10mAH
int16   TCURR       # 转向电流
int16   LCURR       # 左驱动电流
int16   RCURR       # 右驱动电流
uint8[8] UL         # 超声波阵列值
