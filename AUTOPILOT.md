# AUTOPILOT — WindTunnelSuite 自动化推进手册

> 本手册是自动化会话（Cron/无人值守）的**唯一入口**。每次自动推进：读本手册 → 按 §3 队列挑任务 → 按 §4 规程执行 → 按 §5 记录。
> 人工维护者：曼波。自动化执行者：ZCode 会话（无历史上下文，全靠本文件 + devlog + 代码自包含）。

## 0. 铁律（每次必读）

1. **两机边界**：本套件在台式机（`D:\WindTunnelSuite\`）。**禁止 clone/改动 qianli**；FanSim3D 只读调用（addpath），不修改其文件。
2. **单求解槽**：WSL (Ubuntu-24.04) 7.9GB 内存 / 10 核。**同一时间只跑一个 foamRun**；求解期间禁止并发 foamToVTK/reconstructPar（OOM 静默杀进程）。导出/后处理在求解结束后做。
3. **WSL VM 生命周期**：会话间隙 VM 会重启（/tmp 清空、后台进程死）。**长跑求解必须作为后台任务的前台进程**（`wsl -d Ubuntu-24.04 -e bash -c "..."` 的 run_in_background 任务内直接跑 mpirun，不要 nohup 脱离）。长产物放 `~/`，不放 `/tmp`。
4. **数据对≠渲染对**：渲染输出先自查（坐标范围、色标、掩膜方向），再交付。histcounts2 掩膜方向为 nx×nz（行=X），与 imagesc 的 nz×nx 相反需转置（`slices_of.m` 已实现正确版，勿改回）。
5. **数字必须实测提取**：devlog 与报告中的 Q/速度/力全部从日志文件提取，禁止凭记忆。
6. **编码**：MATLAB 工具链脚本（render_of/slices_of 等）输出与注释遇中文经 shell 往返会乱码——新增渲染/批处理脚本注释用中文可以，但 **fprintf 输出与字符串拼接用 ASCII**；用 python 改 MATLAB 文件时字符串含 `\n` 字面量一律用 chr(92)+'n' 拼接。
7. **自动会话结束前**：更新 §3 队列状态 → 写当日 devlog（devlog skill）→ 若有需曼波决策的事项写入 §6。
8. **数据分层**（2026-10-14）：大体积运行数据（vtk 快照等）跑完即 `bash wt/wt.sh push <dir> --hash` 归档到 NAS vault；**求解/渲染只读写本地盘，禁止把 scratch 放 NAS/UNC 路径**；删除类操作仅限 `wt rm --yes`（本地）/`wt vault-rm`（点名确认）。手册：docs/data-vault.md。

## 1. 环境初始化（每次会话开工前执行）

```bash
# Windows 侧: MATLAB R2026a 在 PATH。WSL 侧 (MRF/v12 管线):
wsl -d Ubuntu-24.04 -e bash -c "
unset FOAM_ETC WM_PROJECT_DIR          # 必须先清 v1912 残留, 再 source v12
source /opt/openfoam12/etc/bashrc
cd ~/openfoam/windtunnel-mrf"
# decomposeParDict 用 scotch (v12 的 hierarchical 需要 order 键, 已弃用该格式)
```
- 基线 case (`windtunnel-base`) 若需运行：用 **v1912** + `FOAM_ETC=/usr/share/openfoam/etc WM_PROJECT_DIR=/usr/share/openfoam`（与 v12 冲突时分开两个会话跑）。
- 数据 vault 体检：`bash wt/wt.sh doctor`（未配置 NAS 会给接线指引，见 docs/data-vault.md；vault 未配置时照常干活，只是别删本地 vtk 原件）。
- case 真源在 `D:\WindTunnelSuite\openfoam\windtunnel-mrf\`（生成器输出），WSL 侧 `~/openfoam/windtunnel-mrf/` 是运行副本。改了生成器后需重建：MATLAB 跑 `gen_openfoam_mrf`（开关 `WT_MRF_OBJ=sphere|cylinder` 控制被测物）→ 同步到 WSL。

## 2. 当前项目状态快照（2026-10-06 #3 轮后）

- **960rpm MRF 基线（无被测物）**：✅ Q=0.61±0.02 m³/s，试验段 vol-avg Ux=10.1-10.9 m/s（粗网格 MRF 下界，FluidX3D 动叶轮参照 22.0）。
- **1180rpm（T2 终判收敛 ✓）**：Q=0.7522±0.0162 / Ux=12.782±0.269（风机律 0.750/12.91，双锚点互证 Q∝n）。**稳定图非单调：{790✗@1300, 960✓, 1080✗@2862, 1180✓@3000+}——1080 是特定转速失稳窗，"960 唯一收敛"旧裁决作废**。
- **被测物 sphere @960（T1）**：全链路通；健康窗 Q=0.6106/tsMean=10.37/Cd=0.113（Re≈4.1e4，准无粘欠阻力定性）；~1720 步软发散（球阶梯 +4 skew faces）。
- **渲染管线**：✅ slices_of（掩膜已修+双轮视觉验收 PASS，像素 LUT 反演 vs FO 体平均互证 12.8/12.78）；slice_sweep.m GIF 链✓；**orbit_of.m 整体环绕✓（风洞本体+叶轮+三层等值面+纵切板, 36帧 mp4+gif, 视觉验收 PASS）**；**transient_mov.m 瞬态动态视频✓（叶轮按真实 ω 合成旋转+瞬态流动演化, 1180rpm 演示片已交付 3/3 PASS）**。foamToVTK 必须 `-ascii` + 先 `foamPostProcess -func writeCellCentres -time <t>`（多时刻导出用冒号区间 `-time '3000:'` + `-useTimeName`，C 场覆盖全部导出时刻）。**渲染产物归档 `renders/of/<日期_内容>/`，禁止放 case 内**（生成器 rmdir 连删）。MATLAB 图形层坑见 devlog 2026-10-06 两条（histcounts3 没了/accumarray 下标矩阵/surface texturemap ZData/exportgraphics 紧裁剪帧尺寸/rgb2ind 双输出）。
- **瞬态 MRF 能力（2026-10-06 打通）**：case 配置已瞬态化（backward+PIMPLE+Final 解算器条目），从稳态场热启 0.05s×374 步无发散。**v12 瞬态重启三坑见 devlog 18:07 条**（uniform/time 存的 deltaT 覆盖 controlDict / PIMPLE 要 pFinal+*Final / foamToVTK 冒号区间+useTimeName）。演示解 maxCo 10+PISO(nOuter 1)；**定量瞬态的 dt 被 skew 尖峰格钳在 ~7e-5（Co max 11）——T3 清理后成本可降 ~6 倍**。
- **已知病灶**：skew faces 已清零（Mesh OK 首达，Q 震荡带 σ 减半 0.008）；**170 m/s 尖峰=叶尖/MRF 区界面伪影**（r≈0.18 排线定位，skew 清理无效证明非网格质量 issue）——积分量无污染，处置候选挂 §7（MRF 区半径放大/界面加密/接受）。
- **Q 差距主攻方向**（0.61/0.75 vs 1.38）：瞬态旋转（方案见 docs/transient_rotation_plan.md）> 网格加密 > 真叶片厚 3mm。

## 3. 任务队列（按优先级；完成一项勾一项并写状态）

| # | 任务 | 状态 | 规程 |
|---|---|---|---|
| T0 | 「全盒保留」排查 | ✅ 2026-10-06 结案：真凶=v12 reconstructParMesh 空壳（dict 无罪），修复=reconstructPar -noFields，详见 §4.2 | §4.2 |
| T1 | 被测物 sphere 实测 | ✅ 2026-10-06：全链路通；健康窗 Q=0.6106/tsMean=10.37/Cd=0.113；~1720 步软发散（+4 skew faces）— 重跑验证挂 T3 后 | §4.1 |
| T2 | 1180rpm 长跑补完 | ✅ 2026-10-06：3000 步收敛（过 1080 爆点 2862）Q=0.7522±0.016/Ux=12.78，风机律双锚点互证；**稳定图非单调修订**；10-06 深夜 1200 步 warm start 复核 Q=0.746-0.766 一致（双跑互证闭环） | §4.3 |
| T3 | skew faces 清理 | ✅ 2026-10-06 #4 轮收官: sphere 档重验完成（960rpm 2000 步, T3 收紧版 mesh）— Q=0.611/tsMean=10.39 与健康窗一致, **尖峰 170.7 仍在 → 界面伪影归因最终确认**（与转速/球档无关）; skew 清理收益=震荡带减半+Mesh OK, 定量可信度提升; 残留伪影处置挂 §7 | §4.3 |
| T4 | 试验段横截面切片掩膜方向修复 | ✅ 2026-10-06：cnt2 转置修复+非对称网格单元测试实证+视觉验收 PASS | §4.4 |
| T5 | GIF 输出：切片扫动动画 | ✅ 2026-10-06：slice_sweep.m 新增（114 帧 GIF 产出于 renders/of/2026-10-06_1180rpm/） | §4.5 |
| T6 | Q 差距主攻 A：瞬态旋转可行性调研 | ✅ 2026-10-06：`docs/transient_rotation_plan.md`（两阶段：瞬态 MRF → createNonConformalCouples 真滑移；overset 无教程生态排除；核心洞察=790/1080 发散可能是稳态解存在性问题，瞬态可解锁扫掠） | §4.6 |
| T7 | 3mm OF 交叉验证可行性评估（.wslconfig 扩内存 → 全域加密档位测算） | ⬜ | §4.7 |
| T8 | 全量代码与结构审查 | 🔶 定向预审完成（probe_flow P1 坐标错位已修、sphere_run 弃用标注、extract_sweep legend 修复、fixline/.broken 归档）；剩 gen_openfoam_mrf.m 本体全量审计 | — |
| T9 | 遗留数据 NAS 迁移（~101 GiB，wt migrate 计划已出） | ⬜ 等 §6 两项决策（NAS 挂载 / vtk_old_inline 处置）后 `bash wt/wt.sh migrate --exec` | docs/data-vault.md §2 |

**完成 T1-T4 后**（预计全部核心管线闭环），回到本表追加下阶段任务（头脑风暴产出，见 §7）。

## 4. 任务规程

### 4.1 T1 被测物 sphere 实测
1. 生成器已输出含 `testobj.stl` 的 case（D 盘）。同步：`rm -rf ~/openfoam/windtunnel-mrf && cp -r /mnt/d/... ~/openfoam/`。
2. mesh 阶段按 §1 环境：surfaceOrient×2 → blockMesh → decomposePar(6) → snappy(-np 6, -overwrite) → reconstructParMesh → checkMesh（门槛：体积 0.60-0.80，patch 含 **testobj**）→ topoSet+setsToZones(-noFlipMap)。
3. solve 阶段：decomposePar(10, scotch) → `mpirun -np 10 foamRun -parallel`（endTime 2000，前台跑）。
4. 判定：forces FO 输出在 `postProcessing/forces/.../forces.dat`（Cd/Cl）；试验段 tsMean 应仍 ≈10 m/s 量级（球堵塞 1.6% 影响小）；试验段切片图球后方应见尾迹。
5. 出图：`slices_of('VTK/windtunnel-mrf_XXXX.vtk')`；forces 数据表写入 devlog。

### 4.2 T0（已结案 2026-10-05）「全盒保留」真根因 = v12 reconstructParMesh 空壳
**dict 从头到尾没有问题**（上轮"dict 漂移"假设被推翻）。实证链：失败跑 log.snappy 显示 castellation 全程正常（subset 665696 格、locationInMesh 命中、Finished without errors），但 v12 的 `reconstructParMesh` 被取代为空壳 stub（日志原话 superseded），case 级 polyMesh 仍是 blockMesh 背景 505080——下游所有验证看的都是旧网格。**修复 = `reconstructPar -constant -noFields`**（已进 run_mrf.sh）。修复后 mesh 阶段 669718 格/0.6885 m³ 全门槛通过。
连带修复：① 生成器 regexprep 无 `^` 锚命中 stopAt 行内 endTime（粘连）；② controlDict 模板孤儿 `);` 残骸；③ run_mrf.sh blockMesh 行损坏重定向；④ mesh 阶段自动搬 `tri/*.stl → constant/triSurface/`；⑤ solve 阶段重写 10 进程 scotch dict（原用 mesh 的 6 进程 dict 必炸）；⑥ snappy 模板清成无 testobj 基底，注入为唯一来源（旧"擦除舞蹈"删除）。诊断工具 diag_reconstruct.sh 留档。

### 4.3 T2 1180 长跑
`run_mrf.sh solve 1180 2000 startTime`（冷启动）。判定：末 30% 样本均值±σ；发散（|U|→1e3+）则记录"1180 发散"入表；收敛则记录 Q/Ux。**790 已判发散勿重跑**（除非 T3 清理后重验）。**注意（2026-10-06）：case 当前是 OBJ=sphere 档——T2 要的是无球基线数据，先 `WT_MRF_OBJ=none` 重生成+同步 WSL 再跑；除非明确要"球+1180"组合数据点。**

### 4.3 T3 噪声点清理（方案 A 已进生成器，待验证跑）
病灶 = checkMesh 报的 highly-skew faces（2026-10-06 实测 7 个/max 4.0036，其中 4 个来自球面阶梯）。**根因：etc meshQualityDict.cfg 默认 `maxBoundarySkewness 20`，而 checkMesh 判据是 4——snap 阶段放任边界面歪到 20。**生成器已写入覆盖：`maxBoundarySkewness 4; maxInternalSkewness 3;`（gen_openfoam_mrf.m meshQualityDict 块）。**验证规程**：① 重生成+同步+mesh → checkMesh 目标 **0 highly-skew faces**（snap 提前收敛/贴体变差是可接受代价，看体积门槛）；② 960 冷启动 2000 步 → Q 对比基线 0.61±0.02（**变化 >5% = 病灶确实影响结果，必须记录**）；③ sphere 失稳点重验（重生成 sphere 档 mesh+solve，看发散起点是否从 ~1720 推迟/消失）。若方案 A 无效，备选：collapseEdges 局部处理。

### 4.4 T4 试验段横截面切片（✅ 2026-10-06 完成）
原 pending 掩膜方向 bug 已修：`V2(cnt2'==0)`（histcounts2 ny×nz vs meshgrid nz×ny，numel 相同静默错向）。方向实证法：非对称网格（y 2 桶×z 3 桶）+ 单点探针，cnt(1,2) 非零证明第一维=y。视觉验收确认彩色区单块连续填满 0.30×0.20 绿框。

### 4.5 T5 GIF
对收敛场：沿 x 从 -1.0 到 2.4 步进 3cm，每帧画 y=0 切片云图+当前位置竖线 → `imwrite` 帧 → `gif`（MATLAB）或 ffmpeg 合成。输出 `slice_sweep.gif`。

### 4.6 T6 瞬态旋转调研
产出 `docs/transient_rotation_plan.md`：pimpleFoam+dynamicMesh（AMI 滑移网格）vs MRF 瞬态 vs overlappingMesh 的方案对比、网格/算力需求（7GB WSL 约束）、预期精度收益、实施步骤。只写方案不动代码。

### 4.7 T7 3mm OF 评估
`C:\Users\nebula\.wslconfig` 设 `memory=24GB` 后 `wsl --shutdown` 重启生效（需确认宿主机物理内存 ≥32GB）。全域 3mm ≈ 30-40M 格（不可行）；涵道+试验段 4mm ≈ 4-6M（可行）。写评估到 §6。

## 5. 记录规范

- 每完成一个任务：`devlog/YYYY-MM-DD.md` 新条目（变更/实验数字/结论/待办）。
- 求解日志统一放 case 的 `logs/`，命名 `log.<what>`。
- 数字从日志实测提取（grep outletQ/tsMean 末段均值），禁止估算。
- 需要曼波决策的事项 → 本文件 §6。
- 渲染出图存 `D:\WindTunnelSuite\renders\of\<日期_内容>\`（**禁止放 case 内的 VTK/——生成器 rmdir 重建 case 会连渲染产物一起删，2026-10-06 抢救过一次**）。
- 大体积 run 产物（vtk 快照海）跑完收敛数据提取后：`bash wt/wt.sh push <dir> --hash` 归档，再视空间 `wt rm <name> --yes` 清本地（vault 未配置前跳过 push、保留本地原件）。

## 6. 待曼波决策（QUESTIONS）

- **NAS 接线**：把 NAS 共享映射成盘符（如 `net use Z: \\<nas>\<share>`），`wt/wt.conf` 写 `WT3D_VAULT=Z:/windtunnel-vault`（或告诉我地址我来配）。配好后 `bash wt/wt.sh migrate --exec` 即完成 ~101 GiB 数据上 NAS（计划：70G 现役 vtk → runs/，31G 旧代+渲染源 → archive/）。
- **vtk_old_inline（43G）处置**：P0 修复前损坏快照旧存档，已定性不可信、不迁移。确认无用我就本地删掉（回收 D 盘 43G）；要留也可以原样推 vault archive/。

## 7. 下阶段头脑风暴备忘（种子，供后续生长）

- 170 m/s 尖峰处置（2026-10-06 定性为 MRF 区界面伪影）：候选 = MRF 区半径放大让界面离开叶尖湍流区（改 topoSetDict radius）/ 界面局部加密 / 接受为已知伪影（积分量干净）
- 过球中心线 |U| 剖面工具（2026-10-06 视觉验收建议：球尾迹亏损被收缩段加速背景掩盖，需沿 x 过 (1.25,0,0) 定量采剖面对 Cd/尾迹长度）
- sphere 档 purgeWrite 加大（本次 t=1750 侥幸是失稳前最后健康帧，差一步就被 purge）

- 变频自动寻优：以试验段 25 m/s 为目标，自动二分 rpm（需先解决扫掠稳定性）
- 被测物系列：圆柱/翼型 → Cd-Re 曲线自动化
- 试验段品质指标自动化：Tu、均匀度、边界层厚度从切片场自动提取报告
- 双求解器联合：LBM 高频时序 + OF 稳态收敛，交叉验证自动化
