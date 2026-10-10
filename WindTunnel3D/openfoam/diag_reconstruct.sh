#!/bin/bash
# diag_reconstruct.sh — T0 诊断工具 (2026-10-05 已结案, 留档勿盲目重跑)
# 结论: v12 的 reconstructParMesh 是空壳 stub — "after recm: constant nCells" 永远不变,
# 而 "after snappy: proc0 nCells" 是真网格。修复 = reconstructPar -constant -noFields
# (已进 run_mrf.sh)。本脚本保留用于未来类似"网格丢失"问题的快速定位。
unset FOAM_ETC WM_PROJECT_DIR
source /opt/openfoam12/etc/bashrc
cd ~/openfoam/windtunnel-mrf
polysize() {
  python3 -c "
import re
try:
    s = open('$1/owner').read()
    m = re.search(r'nCells:\s*(\d+)', s)
    print(m.group(1) if m else 'none')
except Exception as e:
    print('ERR', e)"
}
rm -rf processor* constant/polyMesh
blockMesh > logs/log.blockMesh 2>&1
echo "after blockMesh: constant nCells=$(polysize constant/polyMesh)"
decomposePar -force > logs/log.dec6 2>&1
echo "after decompose: proc0 nCells=$(polysize processor0/constant/polyMesh)"
mpirun --allow-run-as-root --oversubscribe -np 6 snappyHexMesh -parallel -overwrite > logs/log.snappy 2>&1
echo "snappy exit=$?"
echo "after snappy: proc0 nCells=$(polysize processor0/constant/polyMesh)"
grep -c "Finished meshing" logs/log.snappy
reconstructParMesh -constant -mergeTol 1e-6 > logs/log.recm 2>&1
echo "after recm(stub!): constant nCells=$(polysize constant/polyMesh)"
reconstructPar -constant -noFields > logs/log.recnofields 2>&1
echo "after reconstructPar(-noFields): constant nCells=$(polysize constant/polyMesh)"
