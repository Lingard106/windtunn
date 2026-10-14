# 数据分层与 NAS Vault（wt 工具）

> 2026-10-14 重构定稿。原则：**本地只留运算端，数据大头住 NAS，求解读写永不落 SMB。**

## 1. 架构

```
┌─ 本地 (台式机/笔记本) ──────────────────┐   ┌─ NAS vault (WT3D_VAULT) ─────────┐
│                                        │   │                                  │
│  <suite>/            代码仓 (git)      │   │  <vault>/windtunnel-suite/       │
│    WindTunnel3D/     求解器/渲染/生成器 │   │    runs/      现役解归档          │
│    FanSim3D/         风机单机           │   │    archive/   旧代/冒烟/渲染源    │
│    fluidx3d/vtk/  ←─ 本地 scratch ──┐  │   │    git/       bare 代码仓 (remote│
│    *.raw *.bin      运算输入(留本地) │  │   │               "nas", 供 clone)   │
│                                     │  │   │                                  │
│  wt push ───────────────────────────┼───┼→ staging 原子改名 → runs/<name>   │
│  wt pull ←──────────────────────────┼───┼── 只读访问                        │
└────────────────────────────────────┘   └──────────────────────────────────┘
```

- **本地 scratch** = `WindTunnel3D/fluidx3d/vtk/`（管线既有约定，路径零改动）。求解器/渲染器只读写本地盘——SMB 上的长时随机小 IO 和锁文件是 NAS 杀手，也是长跑事故源。
- **vault** = NAS 上的唯一真本。每个 run 一个目录 + `MANIFEST.txt`（文件清单/字节数/源路径/机器/时间）+ 可选 `MANIFEST.sha256`。
- **算完即推**：`wt push` → 校验放心后 `wt rm --yes` 清本地 → 要回看时 `wt pull` 秒回。

## 2. 一次性接线（NAS 到位后 10 分钟）

```bash
# ① 挂载共享（资源管理器映射 Z: 或命令行）
net use Z: \\<nas地址>\<共享名> /persistent:yes

# ② 配置 vault 根（或设 Windows 用户环境变量 WT3D_VAULT，两机各自指同一共享）
cd /d/WindTunnelSuite
cp wt/wt.conf.example wt/wt.conf
# 编辑 wt/wt.conf:  WT3D_VAULT=Z:/windtunnel-vault

# ③ 体检
bash wt/wt.sh doctor --sizes

# ④ 先看计划，再执行（可断点续传：中断后原样重跑即续）
bash wt/wt.sh migrate                 # dry-run，打印每项体积与去向
bash wt/wt.sh migrate --exec          # 复制 ~101 GiB（千兆网约 2-3 小时）
bash wt/wt.sh list --all              # 核对
bash wt/wt.sh verify wt3d-vtk_<ts> --hash   # 重要 run 全量校验和（可选但推荐）

# ⑤ 校验放心后回收本地（--purge-local 自动逐项 verify 后才删原件）
bash wt/wt.sh migrate --exec --purge-local --yes
```

`--exec` 与 `--purge-local` 可分开跑也可合并；`--only <id>` 支持单搬一项。

## 3. 日常流

```bash
# 跑完一次求解（vtk 目录出了新快照）：
bash wt/wt.sh push vtk --hash         # 现役解 → runs/vtk_<时间戳>
# ...等数据提取/渲染收尾确认不再需要本地副本后：
bash wt/wt.sh rm vtk_<时间戳> --yes   # 只删本地；vault 副本不动

# 回看历史解：
bash wt/wt.sh list                    # 看库存
bash wt/wt.sh pull vtk_<时间戳>       # 默认还原到当初的源路径（fluidx3d/vtk/）
explore_gpu                            # 无缝继续用

# 媒体成果（renders/ 媒体、报告）不在此流程内 —— 进 git。
```

## 4. 命令参考

| 命令 | 作用 | 删除风险 |
|---|---|---|
| `wt doctor [--sizes]` | 环境/vault 体检 | 无 |
| `wt list [--all] [pattern]` | 库存清单（--all 含 archive） | 无 |
| `wt du <dir>` | 本地目录体积 | 无 |
| `wt push <dir\|速记> [--hash] [--archive]` | 本地→vault，staging 完成才改名 | 无（只增） |
| `wt pull <name> [dest] [--force]` | vault→本地 | 目标非空时需 --force |
| `wt verify <name> [--hash]` | 校验 vault 副本（尺寸级/全量 SHA256） | 无 |
| `wt rm <name> --yes` | 删**本地**副本 | 本地 |
| `wt vault-rm <name> [--yes]` | 删 vault 归档 | vault（交互须点名确认） |
| `wt migrate [--exec] [--purge-local --yes]` | 遗留数据搬迁 | --purge-local 才删本地 |
| `wt git-remote init` | vault 建 bare 仓库 + 挂 remote "nas" | 无 |
| `wt selftest` | 回环自测（临时 vault） | 无 |

速记：`vtk` / `vtk_prev` / `vtk_oldmix` / `fansim-vtk` / `fansim-vtk-prev` / `fansim-vtk-smoke`，其余直接给路径。MATLAB 里用 `wt('list')` / `wt('pull','vtk_...')`（见 `wt/wt.m`）。

## 5. 数据分类定稿

| 数据 | 去向 | 理由 |
|---|---|---|
| `fluidx3d/vtk*/`（几百 MB/帧的快照海） | vault | 数据大头，110G+ |
| `renders/of/*/vtk/`（渲染数据源） | vault | 与媒体成品分离，媒体留 git |
| `*.raw` / `fan_blade.bin` / `*.exe` / `src/` | **本地** | 运算输入与编译物，可由生成器重出 |
| `gpu_meta*.mat` `flow3d_result.mat` 等小 mat | 本地（想保险可 push） | 渲染器直接引用，MB 级 |
| `renders/` 媒体（mp4/gif/png） | git 仓库 | 成果即文档 |
| `openfoam/` case 产物（376M） | 本地（跑完可 push 归档） | WSL 侧运行，体积小 |
| `vtk_old_inline/`（43G） | **不迁移** | P0 修复前损坏快照（2026-09-30 结论），本地删除候选，等曼波点头 |

## 6. NAS 生态隔离守则（wt 内建，非君子协定）

1. **命名空间锁死**：一切写入限定 `<vault>/windtunnel-suite/` 下，所有落点过 `under_vault` 前缀断言，越界即拒；vault 根目录永不触碰。
2. **双向隔离**：vault 在套件内、或套件在 vault 内 → 直接拒绝（防递归复制、防伪 vault）。
3. **默认只增不删**：push/pull/migrate 无任何删除路径；删 vault 仅 `vault-rm` 显式点名（非交互必须 `--yes`）。
4. **不搬运权限**：`/COPY:DT`（只带数据+时间戳，不带 ACL/属主/审计），NAS 侧权限体系不被 NTFS 信息污染。
5. **不跟随链接**：`/XJ` 排除 junction/symlink，防递归环、防把本地目录树拖进 NAS。
6. **脏文件排除**：`.git/`、`Thumbs.db`、`desktop.ini`、`~$*`、`*.tmp`、`*.asv` 一律不进 vault。
7. **命名与原子性**：run 名强制小写 `[a-z0-9._-]`（防 NAS 大小写歧义与非法字符）；先写 `.staging-*` 完成后原子改名，`wt list` 永不显示半成品。
8. **空间预检**：push 前查 vault 剩余空间，不足即中止，不把 NAS 写爆。

## 7. 两机协作（笔记本侧）

```bash
# 笔记本接同一个 NAS 共享，配同一 WT3D_VAULT（wt/wt.conf 各机一份）
bash wt/wt.sh list                       # 台式机推的库存直接可见
bash wt/wt.sh pull vtk_<ts> D:/wt-analysis/vtk   # 显式 dest 拉到笔记本本地分析
bash wt.bat                              # 或双击（cmd 侧入口）

# 代码分发：vault 里的 bare 仓库
bash wt/wt.sh git-remote init            # 台式机：建仓+挂 remote "nas"
git push nas main                        # 部署
git clone <vault>/git/windtunnel-suite.git   # 笔记本/新机：拉取
```

## 8. FAQ

- **为什么不把 scratch 直接放 NAS（junction 指过去）？**
  FluidX3D/foamToVTK 是高频小 IO + 大量目录项，SMB 延迟会让 40 分钟的求解拖成几小时，断连=长跑报废，锁文件残留还会弄脏 NAS。架构上定为：**算在本地，住在 NAS。**
- **push 中断了怎么办？** 原样重跑即续（robocopy 增量跳过已传文件；migrate 自动复用同名 staging）。push 默认命名带时间戳，中断续传建议显式 `--name`。
- **vault 上 manifest 丢了/被 NAS 侧清理动过？** `wt verify` 会立刻报尺寸/哈希不符；这也是重要 run 建议 `--hash` 归档的原因。
- **`vtk_old_inline` 那 43G 怎么处理？** 2026-09-30 P0 结案：快照镜像损坏时代的旧存档，数据不可信。默认不迁移；确认无用后本地直接删即可（`wt rm --path ... --yes` 或资源管理器），不占 NAS 一字节。
