# 固件参考示例交接

整理日期：2026-10-21。此目录是配套移植说明的参考代码；原固件工程不在这里，示例尚未组成可链接、可烧录并经实板验收的完整工程。

完整背景和已有检查记录见[控制策略与 OpenMV / STM32 移植说明](../控制策略与OpenMV_STM32移植说明.md)，尤其第 9 节。仿真代码与运行入口见[仿真目录](../guidance_sim/README.md)。

## 文件责任

| 文件 | 已有内容 | 交接边界 |
|---|---|---|
| [openmv_guidance_example.py](openmv_guidance_example.py) | OpenMV 图像测量、ROI 重捕获修复、确认/丢失状态、原始像素和 24 字节 v2 测量包 | 使用 OpenMV 的 `sensor`、`time`、`pyb.UART`；主机导入只能检查纯函数，不能代替相机运行 |
| [stm32_control_example.h](stm32_control_example.h) | 控制输入输出、两轴自驾仪、测量帧和解析器接口 | 声明不含自身导航、目标观察器、完整弹道预测内核和真实 PWM 驱动 |
| [stm32_control_example.cpp](stm32_control_example.cpp) | 已有过载 PID、滚转 P、四舵混控/限幅、示例脉宽转换和流式解析 | C++ 标准库依赖为 `<cmath>`、`<cstring>`、`<cstddef>`、`<cstdint>`；通道、零位、比例是示例假设 |
| [stm32_task_integration_example.cpp](stm32_task_integration_example.cpp) | 循环 DMA 增量消费和任务接入框架 | 三个 `Platform_*` 外部接口没有在本目录实现，必须由实际工程提供 |

`__pycache__/` 是本机 Python 缓存，不属于源文件或交付依赖。目录内没有独立构建工程、依赖锁文件、测试源码或烧录产物。

## 与现有系统的关系

- 这些文件不被桌面仿真入口直接导入；仿真、参考示例和原板上固件是三个需要区分的部分。
- OpenMV 示例与配套 C++ 解析器使用同一 v2 测量格式；原说明指出旧 7/10 字节解析器不兼容。示例启动即测量，完整 READY / 离架握手未实现。
- 图像时间使用软件采图窗口中点占位，尚未完成曝光延迟标定和两端对时；矩形中心与桌面连通域质心也不完全相同。
- `Platform_PublishVisionFromIsr`、`Platform_ReadControlInput`、`Platform_WriteServoPwm` 是明确的工程缺口，不应将它们的声明误认为已有硬件适配。

## 已有验证与未验证项

移植说明第 9.5 节记录过对象交叉编译、C++ / Python 数值与协议对照、拆包/坏帧/序号回卷及 OpenMV 纯函数检查。本次整理只核对文件与说明，没有重新编译或运行测试。

原记录的测试源码和结果位于本机 `guidance_sim/outputs/traditional_limit/firmware_doc/`，该路径被忽略规则排除，交接时需要另行确认是否包含。已有记录不包含完整固件链接、烧录、实际相机、USART DMA、任务实时性、导航或舵机闭环验收。

后续实现应继续以原移植说明为需求依据；此 README 只登记现有文件责任和缺项，没有新增控制方案。
