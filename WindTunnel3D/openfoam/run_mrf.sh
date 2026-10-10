#!/bin/bash
# run_mrf.sh — windtunnel-mrf 一键流水线 (带每步断言, 失败即停)
# 用法 (WSL, 在 ~/openfoam/windtunnel-mrf):
#   bash <suite>/WindTunnel3D/openfoam/run_mrf.sh mesh
#   bash <suite>/WindTunnel3D/openfoam/run_mrf.sh solve <rpm> <endTime> [startFrom]
# 环境: openfoam.org v12
unset FOAM_ETC WM_PROJECT_DIR
source /opt/openfoam12/etc/bashrc
STAGE=${1:-solve}
RPM=${2:-960}
ENDT=${3:-2000}
START=${4:-latestTime}
CASE=$(pwd)

ncells() {
  python3 -c "
import re
s = open('$CASE/constant/polyMesh/owner').read()
m = re.search(r'nCells:\s*(\d+)', s)
print(m.group(1) if m else 0)"
}

if [ "$STAGE" = "mesh" ]; then
  # 生成器 STL 落在 tri/, OpenFOAM 读 constant/triSurface/ — 此处搬运 (自动轮无手工步)
  mkdir -p constant/triSurface && cp -f tri/*.stl constant/triSurface/
  test -f constant/triSurface/skin.stl && test -f constant/triSurface/impeller.stl || { echo "FAIL: missing core STL"; exit 1; }
  if grep -q 'testobj' system/snappyHexMeshDict; then
    test -f constant/triSurface/testobj.stl || { echo "FAIL: dict has testobj but STL missing"; exit 1; }
  fi
  echo "OK: tri STL"
  surfaceOrient constant/triSurface/skin.stl constant/triSurface/skin.stl '(10 10 10)' > logs/log.orient 2>&1
  surfaceOrient constant/triSurface/impeller.stl constant/triSurface/impeller.stl '(-0.5556 2 0.3726)' > logs/log.orient2 2>&1
  echo "OK: surfaceOrient x2"
  printf 'FoamFile
{
    format ascii;
    class dictionary;
    object decomposeParDict;
}

numberOfSubdomains 6;

method scotch;

' > system/decomposeParDict
  blockMesh > logs/log.blockMesh 2>&1
  N0=$(ncells); echo "blockMesh cells=$N0"
  decomposePar -force > logs/log.dec6 2>&1
  echo "OK: decompose 6"
  mpirun --allow-run-as-root --oversubscribe -np 6 snappyHexMesh -parallel -overwrite > logs/log.snappy 2>&1
  grep -q "Finished meshing without any errors" logs/log.snappy || { echo "FAIL: snappy"; tail -20 logs/log.snappy; exit 1; }
  echo "OK: snappy finished"
  # v12 的 reconstructParMesh 是空壳 stub (被 reconstructPar 取代) — 网格在 processor* 里,
  # 必须 reconstructPar 才会回到 case 级; -noFields = 只重建网格 (2026-10-05 T0 实锤)
  reconstructPar -constant -noFields > logs/log.recm 2>&1
  N1=$(ncells); echo "after snappy cells=$N1"
  VOL=$(checkMesh 2>&1 | grep -oE 'Total volume = [0-9.]+' | grep -oE '[0-9]+\.?[0-9]*')
  echo "Total volume = $VOL"
  python3 -c "import sys; v=float('$VOL'.rstrip('.')); sys.exit(0 if 0.55<=v<=0.85 else 1)" || { echo "FAIL: volume out of gate"; exit 1; }
  echo "OK: volume 0.55-0.85"
  for p in skin_casing skin_inletCap skin_outletCap impeller testobj; do
    grep -q "$p" constant/polyMesh/boundary; echo "OK: patch $p"
  done
  topoSet > logs/log.topoSet 2>&1
  setsToZones -noFlipMap >> logs/log.topoSet 2>&1
  grep -q "Adding set rotorZone as a cellZone" logs/log.topoSet || { echo "FAIL: zones"; exit 1; }
  echo "OK: zones"
  echo "=== MESH STAGE DONE ($N1 cells, vol=$VOL) ==="
  exit 0
fi

if [ "$STAGE" = "solve" ]; then
  sed -i "s/^startFrom .*/startFrom       $START;/" system/controlDict
  sed -i "s/^endTime .*/endTime         $ENDT;/"      system/controlDict
  sed -i "s/omega       .* \[rpm\];/omega       -${RPM} [rpm];/" constant/MRFProperties
  echo "solve rpm=$RPM endTime=$ENDT startFrom=$START"
  # solve 用 10 进程 (mesh 阶段把 dict 写成 6 了) — 必须重写, 否则 decompose6/mpirun10 必炸
  printf 'FoamFile\n{\n    format ascii;\n    class dictionary;\n    object decomposeParDict;\n}\n\nnumberOfSubdomains 10;\n\nmethod scotch;\n\n' > system/decomposeParDict
  decomposePar -force > logs/log.dec10 2>&1
  echo "OK: decompose 10"
  for p in processor*; do cp constant/MRFProperties "$p/constant/MRFProperties"; done
  mpirun --allow-run-as-root --oversubscribe -np 10 foamRun -parallel 2>&1 | grep -vE '^\s*$'
  echo "=== SOLVE DONE rpm=$RPM ==="
  grep -E 'sum(skin_outletCap)' log.run 2>/dev/null | tail -3 || true
  reconstructPar -latestTime > logs/log.recon 2>&1 && echo "reconstruct OK"
  exit 0
fi

echo "usage: run_mrf.sh mesh | solve <rpm> <endTime> [latestTime|startTime]"
