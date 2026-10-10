#!/bin/bash
# sweep_rpm.sh — MRF 转速扫掠 (warm start 链, 须作为任务前台进程跑)
# 注意: 已判 960 唯一收敛, 本脚本仅存档; T2 走 run_mrf.sh solve 冷启动
unset FOAM_ETC WM_PROJECT_DIR
source /opt/openfoam12/etc/bashrc
cd ~/openfoam/windtunnel-mrf
sed -i 's/^startFrom       startTime;/startFrom       latestTime;/' system/controlDict
for rpm in 790 1080 1180; do
  endt=$((5000 + rpm))
  sed -i "s/omega       .* \[rpm\];/omega       -${rpm} [rpm];/" constant/MRFProperties
  sed -i "s/^endTime .*/endTime         ${endt};/" system/controlDict
  echo "=== rpm=$rpm (endTime=$endt) ==="
  grep omega constant/MRFProperties | head -1
  mpirun --allow-run-as-root --oversubscribe -np 10 foamRun -parallel > log.rpm${rpm} 2>&1
  echo "--- rpm=$rpm done: $(grep -cE '^Time = ' log.rpm${rpm}) iters"
  grep -E 'sum\(skin_outletCap\)' log.rpm${rpm} | tail -1
  grep -A1 'tsMean write' log.rpm${rpm} | tail -1
done
echo "=== ALL SWEEP DONE ==="
