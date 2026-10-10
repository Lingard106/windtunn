#!/bin/bash
# sphere_run.sh — sphere 被测物 case 全流程 (自包含; 在 WSL 任意 cwd 调用)
# ⚠️ 2026-10-06 已被 run_mrf.sh 取代 (后者带体积/patch/zone 门槛 + 10 进程 dict 重写 + reconstructPar 修复)。
# 保留仅作历史参考; 新实验一律走 run_mrf.sh mesh | solve
# 用法: bash sphere_run.sh mesh | solve
unset FOAM_ETC WM_PROJECT_DIR
source /opt/openfoam12/etc/bashrc
cd ~/openfoam/windtunnel-mrf || exit 1
STAGE=$1

if [ "$STAGE" = "mesh" ]; then
  blockMesh > logs/log.blockMesh 2>&1 && echo bm-ok
  decomposePar -force > logs/log.dec6 2>&1 && echo dec6-ok
  mpirun --allow-run-as-root --oversubscribe -np 6 snappyHexMesh -parallel -overwrite > logs/log.snappy 2>&1
  echo "snappy: $(grep -c 'Finished meshing without any errors' logs/log.snappy)"
  reconstructPar -constant -noFields > logs/log.recm 2>&1 && echo recm-ok
  checkMesh 2>&1 | grep -E 'cells:|Total volume|Mesh OK|Failed' | head -4
  mpirun --allow-run-as-root --oversubscribe -np 6 topoSet -parallel -dict system/topoSetDict > logs/log.topoSet 2>&1 && echo topo-ok
  mpirun --allow-run-as-root --oversubscribe -np 6 setsToZones -parallel -noFlipMap > logs/log.stz 2>&1 && echo stz-ok
  exit 0
fi

if [ "$STAGE" = "solve" ]; then
  sed -i 's/^startFrom .*/startFrom       latestTime;/' system/controlDict
  sed -i 's/^endTime .*/endTime         2000;/'      system/controlDict
  sed -i 's/omega       .* \[rpm\];/omega       -960 [rpm];/' constant/MRFProperties
  decomposePar -force > logs/log.dec10 2>&1 && echo dec10-ok
  for p in processor*; do
    cp system/fvSolution "$p/system/fvSolution"
    cp constant/MRFProperties "$p/constant/MRFProperties"
  done
  mpirun --allow-run-as-root --oversubscribe -np 10 foamRun -parallel > logs/log.solve 2>&1
  echo "solve iters: $(grep -cE '^Time = ' logs/log.solve)"
  grep -E 'sum(skin_outletCap)' logs/log.solve | tail -1
  grep -A1 'tsMean write' logs/log.solve | tail -1
  exit 0
fi
echo "usage: bash sphere_run.sh mesh | solve"
