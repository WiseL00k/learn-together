# MIT关节电机学习

2026/02/01

## MIT模式是什么

作为一种混合控制，能同时控制电机的位置、速度、扭矩，是并联控制

**核心公式：**

**\$\$T = K\_p(p\_{des} - p) + K\_d(v\_{des} - v) + T\_{ff}\$\$**

其中：

**$T\$**：电机最终输出的力矩（Torque）。

**\$p\_{des}, v\_{des}\$**：期望位置(计数或角度)和期望速度(rpm)

**$p, v\$**：当前实际位置和实际速度（由电机实际反馈）

**$K\_p\$ (刚度系数)**：相当于弹簧的硬度。**\$K\_p\$**越大，电机抗拒位置偏差的力越大，位置控制越硬

**$K\_d\$ (阻尼系数)**：相当于减震器的阻力。**\$K\_d\$**越大，电机对抗速度变化的力越大，运行越平稳，类似于抑制静差

**$T\_{ff}\$ (前馈力矩)**：直接施加的额外力矩，一般用于高输出或者静摩擦与重力补偿

## 如何使用MIT模式驱动关节电机（围绕队内代码）

### 1.MIT模式开启

在使用 SOEM 驱动电机前，电机经过 `INIT -> PRE-OP -> SAFE-OP -> OP` 四个阶段，然后通过POD发送使能指令，电机进入闭环

```
#include "rm_ecat_mit/RmEcatMitSlave.h"

// 1. 创建实例
auto slave = rm_ecat::mit::RmEcatMitSlave::deviceFromFile("path/to/config.yaml", "rm_ecat_slave", 0);

// 2. 启动EtherCAT通信
// 这会进入 PRE_OP -> 自动配置 -> SAFE_OP -> OP
if (slave->startup()) {
    std::cout << "EtherCAT Init Success!" << std::endl;
}
```

## 2.控制循环

```
#include "rm_ecat_mit/Command.h"

void control_loop() {
    // 1. 获取当前的 Command 对象副本
    rm_ecat::mit::Command cmd;

    // 2. 定义控制目标 (MIT 模式 5个参数)
    // 结构体定义在 Command.h 中
    rm_ecat::mit::target target_val;
    target_val.targetPosition = 0.0; // 期望位置 (rad)
    target_val.targetVelocity = 0.0; // 期望速度 (rad/s)
    target_val.kp = 2.0;             // 刚度
    target_val.kd = 0.1;             // 阻尼
    target_val.targetTorque = 0.5;   // 前馈力矩 (Nm)

    // 3. 设置目标给指定的电机
    // 参数: 总线(CAN0/1), 电机ID, 目标结构体
    cmd.setTargetCommand(rm_ecat::mit::CanBus::CAN0, 1, target_val);

    // 4. 将指令放入发送缓冲区 (Stage Command)
    slave->stageCommand(cmd);

    // 5. 执行 EtherCAT 通信，实际发送到硬件
    slave->updateWrite(); 
    slave->updateRead();
}
```

### 3.获取电机反馈

```
// Command.cpp
uint64_t Command::getMotorCommandRaw(CanBus bus, size_t id) const {
  // ... 获取参数 ...
  
  // 1. 浮点转整型 (Factor 是在 ConfigurationParser 里算好的)
  auto tor = static_cast<uint16_t>(torqueFactorNmToInteger_[index] * (targetTorque_[index] - torqueOffset[index]));
  // ... (pos, vel, kp, kd 同理)

  // 2. 手动拼包 (MIT 协议核心)
  uint8_t data[8] = {0};
  data[0] = (pos >> 8);
  data[1] = pos;
  data[2] = (vel >> 4);
  data[3] = ((vel & 0xF) << 4) | (kp >> 8);
  // ... 以此类推
  
  memcpy(&cmd, data, 8);
  return cmd;
}
```

### 在队内 `rm_ecat_dev` 代码：

**MIT 被封装在 `rm_ecat::mit::target` 结构体中。**

**使用 `RmEcatMitSlave::stageCommand()` 存入指令，使用 `RmEcatMitSlave::updateWrite()` 发送。**

**使用 `Command` 类，内部的 `getMotorCommandRaw` 实现标准的 MIT 协议打包**

**填入 PDO** [`RmEcatMitSlave.cpp` - `updateWrite`]:

`rxPdo.can0Commnads_[i] = stagedCommand_.getMotorCommandRaw(...)`

将打包好的 `uint64_t` 放入结构体。

**发送** (SOEM Lib):

SOEM 将 `RxPdo` 结构体通过网线发送给 EtherCAT 网关板。

网关板收到后，拆出 `uint64_t`，将其作为 CAN 报文发送给达妙电机。

#### 初始化链路

**加载配置** [`RmEcatMitSlave.cpp` -> `ConfigurationParser.cpp`]:

系统读取 YAML 文件，确定电机型号（如 `DM4310`）。

**计算系数** [`ConfigurationParser.cpp`]:

根据电机型号，查表得到 `P_MAX`, `V_MAX` 等物理极限

计算转换因子，例如 `positionFactorRadToInteger_`

*逻辑*：`Factor = (2^16 - 1) / (P_MAX - P_MIN)`

**注入参数** [`RmEcatMitSlave::stageCommand`]:

将计算好的Factors和Offset写入到 `Command` 对象中

**意义**：`Command` 对象此时拥有了将 `rad` 转换为 `uint16` 所需的一切数学参数

#### 反馈链路

**接收原始数据** [`RmEcatMitSlave.cpp` - `updateRead`]:

SOEM 从网线收到数据，填入 `TxPdo`。

调用 `reading_.setRawReading(..., txPdo.can0Measurement_[i])`。

**数据解包与转换** [`Reading.cpp`]:

**提取原始值**：`extractPositionCounts` 通过位掩码和移位，从 `uint64_t` 中还原出 `uint16_t` 的原始整数值。

**物理量转换**：`position = raw_int * factor + offset`。此时变回了**浮点数**

**多圈处理** [`Reading.cpp` - `getPosition`]:

因为 CAN 协议里的位置通常只有 16 位（0-65535），电机转一圈数值就会跳变

`Reading` 类维护了一个 `multi_turn_entries` 哈希表

通过对比 `last_code` 和当前值的差异（`delta`），判断电机是否跨过了零点，从而计算出**连续的、多圈的**弧度值

**读取** :

调用 `slave->getReading().getPosition(...)` 获取最终的 `double` 类型位置信息

## 底层握手

在rm_description/urdf/兵种/transmission中,规定了actuator和joint的名字，其中actutor与电机直接对应，joint则是在控制器代码中操作的对象

ecatsalve:

```
stagedCommand_.setMaxOut(CanBus::CAN0, id, motorConfiguration.maxOut_);
    stagedCommand_.setTorqueFactorNmToInteger(CanBus::CAN0, id, motorConfiguration.torqueFactorNmToInteger_);
  }
```

在rm_config下规定bus_id,can_id,电机type,电机name“执行器”

获取tpye之后将相应参数根据


MIT的canid只能配1-6，masterid在此基础上+1，如0X01和0X11.
