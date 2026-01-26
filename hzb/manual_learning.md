# manual学习总结
2026-01-24

## 相关文件作用
rm_manual/include--input_event.h下：
在InputEvent类下封装了遥控信息的状态，比如按下时刻与松开时刻，短按长按的时间区分
rm_manual/src--main.cpp：
入口，为相应机器人创建对象

## 控制流程
流程：1.数据源：manual_base.cpp中，订阅/rm_eact_hw/dbus话题
     2.接受：dbusDataCallback检查数据安全性(时间戳)
     3.解析：InputEvent检测键鼠按下/松开，长短按
     拨杆检测：RC 模式：纯遥控器控制，调用 updateRc。PC模式：键鼠控制，调用 updatePc
     摇杆速度映射（以chassis_gimbal_shooter_manual.cpp的updateRc为例）：右摇杆的dbus_data->ch_r_y是一个-1到1之间的浮点数vel_cmd_sender_->setLinearXVel()将这个比例值转换成底盘的速度指令。
     4.发送：在void ChassisGimbalShooterManual::sendCommand(const ros::Time& time)中打包发送