# 参考来源元数据交接

整理日期：2026-10-21。当前目录只保留 `guidedemo/source_manifest.json`；它是原 Guidedemo 源码的来源与 SHA256 记录，不是完整固件源码归档，也不是本次新增固件示例所在位置。

## 保留内容

[source_manifest.json](guidedemo/source_manifest.json) 记录 2026-10-17 的来源目录 `C:\Users\lingard\Desktop\Guidedemo`，共 16 个文件的相对路径、大小和 SHA256，包括构建入口、导引/IMU任务、PNG、自驾仪 v1/v2、ISMCG、FusionAHRS、BMI088 和测量说明。清单描述当前仿真采用 PNG / Autopilot v1；v2 与 ISMCG 的条目用于来源记录，不代表已选用。

上述 16 个源码文件没有保留在本目录。`source_directory` 是历史机器路径，不是可随工作区复制的依赖。工作区后续加入的 [Guidedemo](../../Guidedemo/README.md) 为另行提供的工程，尚未核验它与这份早期清单是否为同一版本。

## 运行与交接关系

`prototypes/guidedemo_control.py` 的 `SOURCE_MANIFEST` 指向此 JSON，`metadata()` 读取文件哈希并写入仿真来源元数据；读取失败会抛出异常。因此，虽然这里没有固件源码，交付桌面仿真时仍应保留此清单及相对目录。

本目录不参与固件编译和烧录。[firmware_examples](../../firmware_examples/README.md) 是另一个目录中的后续移植参考，二者来源与完成状态不同。原策略及差异分析见[控制策略与 OpenMV / STM32 移植说明](../../控制策略与OpenMV_STM32移植说明.md)。

根 `.gitattributes` 对 `guidance_sim/reference/**` 使用 `-text`，来源文件应保持原始字节，避免文本归一化影响核对。本次没有改动该 JSON；仅检查其格式、条目数及实际目录内容，没有读取历史外部目录或重新验证那 16 个源码哈希。
