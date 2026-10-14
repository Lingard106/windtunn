#!/usr/bin/env bash
# wt.sh -- WindTunnelSuite data-vault tool
#
# 数据分层: 本地仓库只留运算端 (代码/编译物/raw 输入), 大体积运行数据 (vtk* 等)
# 归档到 NAS vault (WT3D_VAULT), 本地 scratch 按需 pull 恢复。
# 求解/渲染永远在本地盘读写, NAS 只做归档与分发 -- 不在 vault 上跑任何求解器。
#
# Usage:
#   wt doctor [--sizes]             环境与 vault 健康检查 (--sizes 附带遗留数据体积)
#   wt list [--all] [pattern]       列出 vault 里的 run 归档 (--all 含 archive/)
#   wt du <dir>                     本地目录体积快查
#   wt push <dir|shorthand> [--name N] [--archive] [--hash]
#                                   本地数据 -> vault (staging 原子改名, --hash 全量校验和)
#   wt pull <name> [dest] [--force] vault -> 本地 (默认还原到 push 时的源路径)
#   wt verify <name> [--hash]       校验 vault 副本 (默认尺寸级; manifest 带 hash 时自动全量)
#   wt rm <name> --yes | wt rm --path <dir> --yes
#                                   删本地副本 (vault 副本不动)
#   wt vault-rm <name> [--yes]      删 vault 归档 (交互须点名确认; 非交互必须 --yes)
#   wt migrate [--exec] [--purge-local --yes] [--only ID]
#                                   遗留数据搬迁: 默认 dry-run 出计划; --exec 复制(可断点续传);
#                                   --purge-local 在校验通过后删本地原件
#   wt git-remote init [url]        在 vault 建 bare 仓库并挂为套件 remote "nas"
#   wt git-remote show              查看当前 nas remote
#   wt selftest                     本地回环自测 (临时 vault, 不碰真实数据)
#
# 隔离守则 (保护 NAS 生态):
#   1. 一切写入锁定在 <vault>/windtunnel-suite/ 命名空间内, 越界即拒
#   2. 默认只增不删: push/pull/migrate 无删除路径; 删除仅 vault-rm 显式点名
#   3. robocopy /XJ 不跟随 junction/symlink, /COPY:DT 不搬运 ACL/属主
#   4. .git/ 临时文件/锁文件/Thumbs.db 一律排除
#   5. run 命名强制小写规范 (防 NAS 大小写歧义), staging 完成才改名 (防半成品)

# ---------------------------------------------------------------- shell setup
export LC_ALL=C
WT_HOME="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SUITE="$(dirname "$WT_HOME")"
NS_NAME="windtunnel-suite"
LOGDIR="$WT_HOME/.logs"
MANIFEST="MANIFEST.txt"
HASHFILE="MANIFEST.sha256"

msg() { printf 'wt: %s\n' "$*"; }
die() { printf 'wt: ERROR: %s\n' "$*" >&2; exit 1; }
now_iso() { date +%Y-%m-%dT%H:%M:%S%z; }
ts() { date +%Y%m%d-%H%M%S; }
hostname_id() { hostname 2>/dev/null || uname -n; }

to_posix() { cygpath -u "$1" 2>/dev/null || printf '%s' "$1"; }
to_win()   { cygpath -w "$1" 2>/dev/null || printf '%s' "$1"; }

no_trailing() { local p="${1%/}"; printf '%s' "${p:-/}"; }

human_bytes() {
  awk -v b="$1" 'BEGIN{
    split("B KiB MiB GiB TiB",u," "); i=1;
    while (b>=1024 && i<5) {b/=1024; i++}
    printf (i==1)?"%d %s":"%.1f %s", b, u[i]}'
}

# stats_of <dir> -> "files bytes"  (排除杂项与 manifest 自身, 与 robocopy 排除口径一致)
stats_of() {
  find "$1" -type f \
    ! -name 'Thumbs.db' ! -name 'desktop.ini' ! -name '~$*' \
    ! -name '*.tmp' ! -name '*.asv' ! -path '*/.git/*' \
    ! -name "$MANIFEST" ! -name "$HASHFILE" \
    -printf '%s\n' | awk '{n++; s+=$1} END{printf "%d %d\n", n+0, s+0}'
}

df_free_kb() { df -P "$1" 2>/dev/null | awk 'NR==2{print $4}'; }

# ---------------------------------------------------------------- vault 配置
load_conf() {
  if [ -z "${WT3D_VAULT:-}" ] && [ -f "$WT_HOME/wt.conf" ]; then
    WT3D_VAULT="$(grep -E '^[[:space:]]*WT3D_VAULT=' "$WT_HOME/wt.conf" \
      | head -1 | cut -d= -f2- | tr -d '\r' \
      | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//' -e 's/^"//' -e 's/"$//')"
  fi
}

# require_vault: 解析 + 连通 + 可写 + 命名空间隔离自检; 成功后 VAULT/NS 全局可用
require_vault() {
  load_conf
  [ -n "${WT3D_VAULT:-}" ] || die "WT3D_VAULT not set.
  setup: (1) map NAS share to a drive letter (e.g. net use Z: \\\\nas\\share)
         (2) create wt/wt.conf containing:  WT3D_VAULT=Z:/windtunnel-vault
         (3) wt doctor"
  VAULT="$(no_trailing "$(to_posix "$WT3D_VAULT")")"
  NS="$VAULT/$NS_NAME"
  [ -d "$VAULT" ] || die "vault not reachable: $VAULT ($WT3D_VAULT)"
  # 隔离自检: vault 不得在套件内, 套件不得在 vault 内 (防递归复制/伪 vault)
  case "$(no_trailing "$NS")/" in
    "$(no_trailing "$SUITE")"/*) die "vault lives inside the suite repo -- refusing (isolation)" ;;
  esac
  case "$(no_trailing "$SUITE")/" in
    "$(no_trailing "$NS")"/*) die "suite repo lives inside the vault -- refusing (isolation)" ;;
  esac
  mkdir -p "$NS/tmp" 2>/dev/null || die "vault not writable: $NS"
  rmdir "$NS/tmp" 2>/dev/null
}

# under_vault <abs-posix-path>: 断言最终落点在命名空间内
under_vault() {
  case "$(no_trailing "$1")/" in
    "$(no_trailing "$NS")"/*) return 0 ;;
    *) die "path escapes vault namespace: $1" ;;
  esac
}

valid_name() {
  printf '%s' "$1" | grep -Eq '^[a-z0-9][a-z0-9._-]*$'
}

# ---------------------------------------------------------------- robocopy
rcopy() { # rcopy <src_posix> <dst_posix> <log_posix>
  local src dst log rc
  src="$(to_win "$1")"; dst="$(to_win "$2")"; log="$(to_win "$3")"
  mkdir -p "$(dirname "$3")"
  # MSYS2_ARG_CONV_EXCL: Git Bash 会把 /E /XJ 等 robocopy 开关当 POSIX 路径转换
  MSYS2_ARG_CONV_EXCL='*' MSYS_NO_PATHCONV=1 \
    robocopy "$src" "$dst" /E /XJ /COPY:DT /DCOPY:T /R:1 /W:1 /MT:8 \
    /NP /NFL /NDL \
    /XF "${XC_FILES[@]}" /XD "${XC_DIRS[@]}" \
    /LOG:"$log"
  rc=$?
  if [ "$rc" -ge 8 ]; then
    printf 'wt: robocopy FAILED (exit %d), log: %s\n' "$rc" "$3" >&2
    return 1
  fi
}
XC_FILES=(Thumbs.db desktop.ini '~$*' '*.tmp' '*.asv')
XC_DIRS=(.git)

# ---------------------------------------------------------------- manifest
# write_manifest <run_dir> <name> <src_posix> <hashmode none|sha256>
write_manifest() {
  local rd=$1 name=$2 src=$3 hm=$4
  local stats
  stats="$(stats_of "$rd")"
  {
    printf '# WTSuite run manifest v1\n'
    printf 'name: %s\n' "$name"
    printf 'source: %s\n' "$src"
    printf 'created: %s\n' "$(now_iso)"
    printf 'machine: %s\n' "$(hostname_id)"
    printf 'files: %s\n' "${stats%% *}"
    printf 'bytes: %s\n' "${stats##* }"
    printf 'hash: %s\n' "$hm"
    printf 'begin-files\n'
    ( cd "$rd" && find . -type f \
        ! -name 'Thumbs.db' ! -name 'desktop.ini' ! -name '~$*' \
        ! -name '*.tmp' ! -name '*.asv' ! -path '*/.git/*' \
        ! -name "$MANIFEST" ! -name "$HASHFILE" \
        -printf '%P\t%s\n' | LC_ALL=C sort )
  } > "$rd/$MANIFEST"
  if [ "$hm" = "sha256" ]; then
    ( cd "$rd" && find . -type f \
        ! -name 'Thumbs.db' ! -name 'desktop.ini' ! -name '~$*' \
        ! -name '*.tmp' ! -name '*.asv' ! -path '*/.git/*' \
        ! -name "$MANIFEST" ! -name "$HASHFILE" \
        -print0 | LC_ALL=C sort -z | xargs -0 sha256sum ) > "$rd/$HASHFILE"
  fi
}

# mf_get <run_dir> <key> -> value (空串若无)
mf_get() {
  grep -m1 -E "^$2: " "$1/$MANIFEST" 2>/dev/null | cut -d' ' -f2- || true
}

# locate_run <name> -> "<runs|archive> <dir>"
locate_run() {
  local d="$NS/runs/$1"
  [ -d "$d" ] && { printf 'runs %s' "$d"; return 0; }
  d="$NS/archive/$1"
  [ -d "$d" ] && { printf 'archive %s' "$d"; return 0; }
  return 1
}

# verify_dir <run_dir> <want_hash 0|1>: 对照 manifest 校验; 返回 0=OK
verify_dir() {
  local rd=$1 want_hash=$2
  local mf_files mf_bytes mf_hash n b
  [ -f "$rd/$MANIFEST" ] || { printf 'wt: no manifest in %s\n' "$rd" >&2; return 2; }
  mf_files="$(mf_get "$rd" files)"; mf_bytes="$(mf_get "$rd" bytes)"
  mf_hash="$(mf_get "$rd" hash)"
  read -r n b <<< "$(stats_of "$rd")"
  if [ "$n" != "$mf_files" ] || [ "$b" != "$mf_bytes" ]; then
    printf 'wt: SIZE MISMATCH in %s: manifest %s files/%s bytes, on-disk %s files/%s bytes\n' \
      "$rd" "$mf_files" "$mf_bytes" "$n" "$b"
    return 1
  fi
  if [ "$want_hash" = 1 ] && [ "$mf_hash" != "sha256" ] && [ ! -f "$rd/$HASHFILE" ]; then
    printf 'wt: --hash requested but this run was archived with hash=none\n' >&2
    return 2
  fi
  if [ "$mf_hash" = "sha256" ] || [ "$want_hash" = 1 ]; then
    local bad=0 checked=0
    while IFS= read -r line; do
      [ -n "$line" ] || continue
      local sum="${line%% *}"
      local rel="${line#* }"      # sha256sum 可能输出 "  ./p" 或 " *./p"
      rel="${rel#\*}"
      rel="${rel# }"
      if ! ( cd "$rd" && printf '%s  %s\n' "$sum" "$rel" | sha256sum -c --quiet - ) 2>/dev/null; then
        printf 'wt: HASH MISMATCH: %s\n' "$rel"
        bad=$((bad+1))
        [ "$bad" -ge 10 ] && { printf 'wt: ...more mismatches suppressed\n'; break; }
      fi
      checked=$((checked+1))
    done < "$rd/$HASHFILE"
    [ "$bad" -gt 0 ] && return 1
    printf 'wt: hash check OK (%s files)\n' "$checked"
  else
    printf 'wt: size check OK (%s files, %s bytes)\n' "$n" "$b"
  fi
  return 0
}

# ---------------------------------------------------------------- shorthand
shorthand_resolve() { # <key|path> -> 路径 (速记展开为 suite 内路径)
  case "$1" in
    vtk)             printf '%s' "$SUITE/WindTunnel3D/fluidx3d/vtk" ;;
    vtk_prev)        printf '%s' "$SUITE/WindTunnel3D/fluidx3d/vtk_prev" ;;
    vtk_oldmix)      printf '%s' "$SUITE/WindTunnel3D/fluidx3d/vtk_oldmix" ;;
    vtk_old_inline)  printf '%s' "$SUITE/WindTunnel3D/fluidx3d/vtk_old_inline" ;;
    fansim-vtk)      printf '%s' "$SUITE/FanSim3D/fluidx3d/vtk" ;;
    fansim-vtk-prev) printf '%s' "$SUITE/FanSim3D/fluidx3d/vtk_prev" ;;
    fansim-vtk-smoke) printf '%s' "$SUITE/FanSim3D/fluidx3d/vtk_smoke" ;;
    *) printf '%s' "$1" ;;
  esac
}
tag_of() { # 默认 tag: 目录名小写、非法字符转下划线
  local b t
  b="$(basename "$(no_trailing "$1")")"
  t="$(printf '%s' "$b" | tr 'A-Z ' 'a-z_' | tr -c 'a-z0-9._-' '_')"
  valid_name "$t" && printf '%s' "$t" || printf 'data'
}

# ---------------------------------------------------------------- commands
cmd_du() {
  local d
  [ -n "${1:-}" ] || die "usage: wt du <dir>"
  d="$(to_posix "$1")"
  [ -d "$d" ] || die "not a directory: $d"
  du -sh "$d"
}

cmd_list() {
  require_vault
  local all=0 pat=""
  local args=()
  for a in "$@"; do
    case "$a" in
      --all) all=1 ;;
      *) args+=("$a") ;;
    esac
  done
  pat="${args[0]:-}"
  local roots=("$NS/runs")
  [ "$all" = 1 ] && roots+=("$NS/archive")
  local have=0 root d name cls f b c
  for root in "${roots[@]}"; do
    [ -d "$root" ] || continue
    for d in "$root"/*/; do
      [ -d "$d" ] || continue
      have=1
      d="${d%/}"
      name="$(basename "$d")"
      cls="$(basename "$root")"
      [ -z "$pat" ] || printf '%s' "$name" | grep -q "$pat" || continue
      if [ -f "$d/$MANIFEST" ]; then
        f="$(mf_get "$d" files)"; b="$(mf_get "$d" bytes)"; c="$(mf_get "$d" created)"
        c="${c%%T*}"
      else
        f="?"; b=""; c="(no manifest)"
      fi
      local hbsz="?"
      [ -n "$b" ] && hbsz="$(human_bytes "$b")"
      printf '%-42s %-8s %10s %12s  %s\n' "$name" "$cls" "$f" "$hbsz" "$c"
    done
  done
  [ "$have" = 1 ] || msg "vault is empty: $NS"
  return 0
}

cmd_push() {
  require_vault
  local src="" name="" want_hash=0 dest_class="runs"
  while [ $# -gt 0 ]; do
    case "$1" in
      --name) name="$2"; shift 2 ;;
      --archive) dest_class="archive"; shift ;;
      --hash) want_hash=1; shift ;;
      *) [ -z "$src" ] || die "push: unexpected arg '$1'"; src="$1"; shift ;;
    esac
  done
  [ -n "$src" ] || die "usage: wt push <dir|shorthand> [--name N] [--archive] [--hash]"
  src="$(no_trailing "$(to_posix "$(shorthand_resolve "$src")")")"
  [ -d "$src" ] || die "source dir not found: $src"
  [ "$(no_trailing "$(to_posix "$src")")" != "$VAULT" ] || die "refusing to push the vault into itself"
  if [ -n "$name" ]; then
    name="$(printf '%s' "$name" | tr 'A-Z' 'a-z')"
    valid_name "$name" || die "bad name '$name' (lowercase [a-z0-9._-] only)"
  else
    name="$(tag_of "$src")_$(ts)"
  fi
  under_vault "$NS/$dest_class/$name"
  [ ! -e "$NS/$dest_class/$name" ] || die "destination exists on vault: $dest_class/$name (use --name)"
  local staging="$NS/$dest_class/.staging-$name"
  under_vault "$staging"
  # 空间预检 (含余量)
  local sstats need_kb free_kb
  sstats="$(stats_of "$src")"
  need_kb=$(( ${sstats##* } / 1024 + 10240 ))
  free_kb="$(df_free_kb "$NS")"
  if [ -n "$free_kb" ] && [ "$free_kb" -lt "$need_kb" ]; then
    die "vault low on space: need ~$(human_bytes $((need_kb*1024))), free $(human_bytes $((free_kb*1024)))"
  fi
  msg "pushing $(human_bytes ${sstats##* }) (${sstats%% *} files)"
  msg "  from $src"
  msg "  to   $NS/$dest_class/$name"
  mkdir -p "$staging"
  if ! rcopy "$src" "$staging" "$LOGDIR/push_${name}.log"; then
    msg "copy failed; staging kept: $staging"
    msg "resume with: wt push '$1' --name $name $([ $want_hash = 1 ] && printf -- '--hash')"
    exit 1
  fi
  local hm="none"
  [ "$want_hash" = 1 ] && hm="sha256"
  write_manifest "$staging" "$name" "$src" "$hm"
  mv "$staging" "$NS/$dest_class/$name" || die "staging rename failed: $staging"
  free_kb="$(df_free_kb "$NS")"
  msg "pushed $name: ${sstats%% *} files, $(human_bytes ${sstats##* }), hash=$hm"
  [ -n "$free_kb" ] && msg "vault free: $(human_bytes $((free_kb*1024)))"
  msg "local original untouched; after you trust the vault copy, free space with: wt rm $name --yes"
}

cmd_pull() {
  require_vault
  local name="" dest="" force=0 a
  while [ $# -gt 0 ]; do
    case "$1" in
      --force) force=1; shift ;;
      *) if [ -z "$name" ]; then name="$1"; elif [ -z "$dest" ]; then dest="$1"; else die "pull: unexpected arg '$1'"; fi; shift ;;
    esac
  done
  [ -n "$name" ] || die "usage: wt pull <name> [dest] [--force]"
  local loc rd
  loc="$(locate_run "$name")" || die "run not found on vault: $name (try: wt list)"
  rd="${loc#* }"
  if [ -z "$dest" ]; then
    dest="$(mf_get "$rd" source)"
    [ -n "$dest" ] || die "manifest has no source path; pass dest explicitly"
  fi
  dest="$(no_trailing "$(to_posix "$dest")")"
  if [ -e "$dest" ] && [ -n "$(ls -A "$dest" 2>/dev/null)" ] && [ "$force" != 1 ]; then
    die "target not empty: $dest (use --force to merge into it)"
  fi
  mkdir -p "$dest"
  local dfree
  dfree="$(df_free_kb "$dest")"
  [ -n "$dfree" ] && msg "target free space: $(human_bytes $((dfree*1024)))"
  msg "pulling $name -> $dest"
  if ! rcopy "$rd" "$dest" "$LOGDIR/pull_${name}.log"; then
    die "pull failed; re-run same command to resume (robocopy is incremental)"
  fi
  msg "pulled $name (vault copy untouched, read-only access)"
}

cmd_verify() {
  require_vault
  local name="" want_hash=0
  while [ $# -gt 0 ]; do
    case "$1" in
      --hash) want_hash=1; shift ;;
      *) [ -z "$name" ] || die "verify: unexpected arg '$1'"; name="$1"; shift ;;
    esac
  done
  [ -n "$name" ] || die "usage: wt verify <name> [--hash]"
  local loc rd
  loc="$(locate_run "$name")" || die "run not found on vault: $name"
  rd="${loc#* }"
  msg "verifying $name ($(basename "$(dirname "$rd")")) on vault..."
  verify_dir "$rd" "$want_hash"
}

cmd_rm() {
  require_vault
  local name="" path="" yes=0
  while [ $# -gt 0 ]; do
    case "$1" in
      --yes) yes=1; shift ;;
      --path) path="$2"; shift 2 ;;
      *) [ -z "$name" ] || die "rm: unexpected arg '$1'"; name="$1"; shift ;;
    esac
  done
  [ "$yes" = 1 ] || die "refusing to delete without --yes (vault copy is never touched by wt rm)"
  if [ -n "$path" ]; then
    path="$(no_trailing "$(to_posix "$path")")"
    [ -d "$path" ] || die "no such dir: $path"
    msg "deleting LOCAL $path (vault untouched)"
    rm -rf "$path" && msg "deleted local: $path"
    return 0
  fi
  [ -n "$name" ] || die "usage: wt rm <name> --yes | wt rm --path <dir> --yes"
  local loc rd src
  loc="$(locate_run "$name")" || die "run not found on vault: $name"
  rd="${loc#* }"
  src="$(mf_get "$rd" source)"
  [ -n "$src" ] && [ -d "$src" ] || die "local source no longer exists ($src); nothing to clean locally"
  msg "deleting LOCAL $src (vault copy of $name stays)"
  rm -rf "$src" && msg "deleted local: $src"
}

cmd_vault_rm() {
  require_vault
  local name="" yes=0
  while [ $# -gt 0 ]; do
    case "$1" in
      --yes) yes=1; shift ;;
      *) [ -z "$name" ] || die "vault-rm: unexpected arg '$1'"; name="$1"; shift ;;
    esac
  done
  [ -n "$name" ] || die "usage: wt vault-rm <name> [--yes]"
  local loc rd
  loc="$(locate_run "$name")" || die "run not found on vault: $name"
  rd="${loc#* }"
  under_vault "$rd"
  if [ "$yes" != 1 ]; then
    if [ ! -t 0 ]; then die "non-interactive without --yes; aborting"; fi
    printf 'wt: type the run name to confirm deletion ON VAULT (%s): ' "$rd"
    read -r ans
    [ "$ans" = "$name" ] || die "confirmation mismatch; aborting"
  fi
  rm -rf "$rd" && msg "deleted on vault: $rd"
}

# ---------------------------------------------------------------- migrate
# 遗留数据搬迁表: <id>|<suite 相对路径>|<目标类 runs/archive>|<备注>
mig_items() {
  cat <<'EOF'
wt3d-vtk|WindTunnel3D/fluidx3d/vtk|runs|current active generation (R1 clean snapshots)
wt3d-vtk-prev|WindTunnel3D/fluidx3d/vtk_prev|archive|older generation
wt3d-vtk-oldmix|WindTunnel3D/fluidx3d/vtk_oldmix|archive|mixed-generation leftovers (2026-10-02 incident)
fansim-vtk-prev|FanSim3D/fluidx3d/vtk_prev|archive|older generation
fansim-vtk-smoke|FanSim3D/fluidx3d/vtk_smoke|archive|smoke-test snapshots
EOF
  local v base="$SUITE/renders/of"
  if [ -d "$base" ]; then
    for v in "$base"/*/vtk; do
      [ -d "$v" ] || continue
      local tag; tag="$(basename "$(dirname "$v")")"
      printf 'renders-of-%s|renders/of/%s/vtk|archive|render input vtk (media outputs stay in repo)\n' "$tag" "$tag"
    done
  fi
  printf 'SKIP wt3d-vtk-old-inline|WindTunnel3D/fluidx3d/vtk_old_inline|SKIP|pre-P0-fix corrupt snapshots -- do NOT archive; local delete candidate (needs Manbo nod)\n'
}

cmd_migrate() {
  require_vault
  local do_exec=0 purge=0 yes=0 only=""
  while [ $# -gt 0 ]; do
    case "$1" in
      --exec) do_exec=1; shift ;;
      --purge-local) purge=1; shift ;;
      --yes) yes=1; shift ;;
      --only) only="$2"; shift 2 ;;
      *) die "migrate: unknown option $1" ;;
    esac
  done
  { [ "$purge" = 0 ] || [ "$do_exec" = 1 ]; } || die "--purge-local requires --exec"
  { [ "$purge" = 0 ] || [ "$yes" = 1 ]; } || die "--purge-local also requires --yes (it deletes local originals)"
  msg "vault namespace: $NS"
  if [ "$do_exec" = 0 ]; then
    msg "DRY RUN plan (add --exec to copy; --exec --purge-local --yes also frees local originals)"
  else
    msg "EXECUTING migration (robocopy incremental: re-run same command to resume any item)"
  fi
  local total_bytes=0
  local id rel destcls note
  while IFS='|' read -r id rel destcls note; do
    [ -n "$id" ] || continue
    local skip=0
    case "$id" in SKIP\ *) skip=1; id="${id#SKIP }";; esac
    if [ -n "$only" ] && [ "$only" != "$id" ]; then continue; fi
    local src="$SUITE/$rel"
    if [ ! -d "$src" ]; then
      printf '  [ ] %-40s (absent: %s)\n' "$id" "$rel"
      continue
    fi
    if [ "$skip" = 1 ]; then
      printf '  [!] %-40s %8s  NOT MIGRATED -- %s\n' "$id" "$(du -sh "$src" 2>/dev/null | cut -f1)" "$note"
      continue
    fi
    local sstats sz
    sstats="$(stats_of "$src")"
    sz="$(human_bytes "${sstats##* }")"
    total_bytes=$(( total_bytes + ${sstats##* } ))
    if [ "$do_exec" = 0 ]; then
      printf '  [ ] %-40s %8s  %-7s -> %s/%s/%s_<ts>\n' "$id" "$sz" "$destcls" "$NS" "$destcls" "$id"
      continue
    fi
    # 断点续传: 复用该条目最近的 staging (若存在), 否则起新名
    local name staging
    staging="$(ls -d "$NS/$destcls"/.staging-"$id"_* 2>/dev/null | sort | tail -1)"
    if [ -n "$staging" ]; then
      name="${staging##*/.staging-}"
      msg "[$id] resuming into existing staging: $staging"
    else
      name="${id}_$(ts)"
      staging="$NS/$destcls/.staging-$name"
    fi
    under_vault "$staging"
    [ ! -e "$NS/$destcls/$name" ] || die "destination exists: $destcls/$name"
    if [ -d "$staging" ]; then
      msg "[$id] resuming into existing staging"
    else
      mkdir -p "$staging"
    fi
    msg "[$id] copying $sz -> $staging"
    if ! rcopy "$src" "$staging" "$LOGDIR/migrate_${id}.log"; then
      die "[$id] copy failed; staging kept, re-run same command to resume"
    fi
    write_manifest "$staging" "$name" "$src" "none"
    mv "$staging" "$NS/$destcls/$name" || die "[$id] staging rename failed"
    msg "[$id] archived as $destcls/$name"
    if [ "$purge" = 1 ]; then
      if verify_dir "$NS/$destcls/$name" 0 >/dev/null 2>&1; then
        msg "[$id] verify OK, purging local original"
        rm -rf "$src" && msg "[$id] local freed: $rel"
      else
        msg "[$id] verify FAILED -- local original KEPT: $rel"
      fi
    fi
  done < <(mig_items)
  if [ "$do_exec" = 0 ]; then
    msg "plan total: $(human_bytes "$total_bytes") to copy"
    msg "note: vtk_old_inline intentionally not migrated (see [!] row)"
  fi
}

cmd_git_remote() {
  require_vault
  local action="${1:-show}"
  local default_url="$NS/git/$NS_NAME.git"
  case "$action" in
    init)
      local url="${2:-$default_url}"
      under_vault "$url"
      mkdir -p "$(dirname "$url")"
      msg "creating bare repo: $url"
      git init --bare --quiet "$url" || die "git init --bare failed at $url"
      ( cd "$SUITE" && git remote remove nas 2>/dev/null; git remote add nas "$url" )
      msg "remote 'nas' added to suite repo"
      msg "deploy:   cd $SUITE && git push nas main"
      msg "elsewhere: git clone $url"
      ;;
    show)
      ( cd "$SUITE" && git remote -v | grep -E '^nas' ) || msg "no 'nas' remote yet (wt git-remote init)"
      ;;
    *) die "usage: wt git-remote init [url] | show" ;;
  esac
}

cmd_doctor() {
  load_conf
  local ok=1 sizes=0
  [ "${1:-}" = "--sizes" ] && sizes=1
  printf 'wt doctor -- %s\n' "$(hostname_id)"
  printf '  suite root : %s\n' "$SUITE"
  if [ -z "${WT3D_VAULT:-}" ]; then
    printf '  vault      : NOT CONFIGURED (WT3D_VAULT unset; wt/wt.conf absent or empty)\n'
    printf '               setup: net use Z: \\\\nas\\share  ->  wt/wt.conf: WT3D_VAULT=Z:/windtunnel-vault\n'
    return 1
  fi
  printf '  vault      : %s\n' "$WT3D_VAULT"
  require_vault
  printf '  namespace  : %s (reachable + writable OK)\n' "$NS"
  command -v robocopy >/dev/null 2>&1 || { printf '  robocopy   : MISSING\n'; ok=0; }
  command -v sha256sum >/dev/null 2>&1 || printf '  sha256sum  : MISSING (--hash disabled)\n'
  local fk
  fk="$(df_free_kb "$NS")"
  [ -n "$fk" ] && printf '  free space : %s\n' "$(human_bytes $((fk*1024)))"
  if [ "$sizes" = 1 ]; then
    printf '  legacy data pending migration:\n'
    local id rel destcls note src
    while IFS='|' read -r id rel destcls note; do
      [ -n "$id" ] || continue
      src="$SUITE/$rel"
      [ -d "$src" ] || continue
      printf '    %-24s %8s  %s\n' "$id" "$(du -sh "$src" 2>/dev/null | cut -f1)" "$destcls"
    done < <(mig_items)
  fi
  [ "$ok" = 1 ] && msg "doctor OK" || { msg "doctor found problems"; return 1; }
}

# ---------------------------------------------------------------- selftest
cmd_selftest() {
  local tmp vault src sh="$WT_HOME/wt.sh"
  tmp="$(mktemp -d "${TMPDIR:-/tmp}/wt-selftest.XXXXXX")"
  vault="$tmp/vault"; src="$tmp/src"
  mkdir -p "$vault" "$src/sub"
  head -c 1048576 /dev/urandom > "$src/a.bin"
  printf 'hello vault\n' > "$src/sub/c.txt"
  printf 'junk\n' > "$src/Thumbs.db"
  printf 'junk\n' > "$src/x.tmp"
  printf 'junk\n' > "$src/foo.asv"
  mkdir -p "$src/.git"; printf 'never\n' > "$src/.git/HEAD"
  local pass=0 fail=0
  t_ok() { if [ "$1" = 0 ]; then printf 'PASS: %s\n' "$2"; pass=$((pass+1)); else printf 'FAIL: %s\n' "$2" >&2; fail=$((fail+1)); fi; }
  w() { WT3D_VAULT="$vault" bash "$sh" "$@"; }

  w push "$src" --name test-run --hash >/dev/null 2>&1
  t_ok $? "push round-trip"
  [ -d "$vault/$NS_NAME/runs/test-run" ]
  t_ok $? "staging renamed into runs/"
  { [ ! -e "$vault/$NS_NAME/runs/test-run/Thumbs.db" ] && \
    [ ! -e "$vault/$NS_NAME/runs/test-run/x.tmp" ] && \
    [ ! -e "$vault/$NS_NAME/runs/test-run/foo.asv" ] && \
    [ ! -e "$vault/$NS_NAME/runs/test-run/.git" ]; }
  t_ok $? "junk excluded (Thumbs.db/tmp/asv/.git)"
  cmp -s "$src/a.bin" "$vault/$NS_NAME/runs/test-run/a.bin"
  t_ok $? "byte-identical push"

  w list 2>/dev/null | grep -q 'test-run'
  t_ok $? "list shows run"

  rm -rf "$src"
  w pull test-run >/dev/null 2>&1
  t_ok $? "pull restores to manifest source"
  cmp -s "$vault/$NS_NAME/runs/test-run/a.bin" "$tmp/src/a.bin"
  t_ok $? "byte-identical pull"

  w verify test-run --hash >/dev/null 2>&1
  t_ok $? "verify hash OK"
  printf 'tamper' >> "$vault/$NS_NAME/runs/test-run/sub/c.txt"
  w verify test-run --hash >/dev/null 2>&1
  [ $? -ne 0 ]
  t_ok $? "tamper detected by verify"

  w push "$src" --name BadName >/dev/null 2>&1
  t_ok $? "uppercase --name auto-lowercased accepted"
  w list 2>/dev/null | grep -q '^badname'
  t_ok $? "run stored under lowercase name"

  mkdir -p "$SUITE/_wt_evil_vault"
  WT3D_VAULT="$SUITE/_wt_evil_vault" bash "$sh" doctor >/dev/null 2>&1
  [ $? -ne 0 ]
  t_ok $? "guard: vault inside suite refused"
  rmdir "$SUITE/_wt_evil_vault" 2>/dev/null

  w vault-rm test-run >/dev/null 2>&1
  [ $? -ne 0 ]
  t_ok $? "vault-rm without --yes refused (non-interactive)"
  w vault-rm test-run --yes >/dev/null 2>&1
  t_ok $? "vault-rm --yes deletes"
  [ ! -d "$vault/$NS_NAME/runs/test-run" ]
  t_ok $? "vault-rm actually deleted"

  rm -rf "$tmp"
  printf 'wt: selftest done -- %d pass, %d fail\n' "$pass" "$fail"
  [ "$fail" = 0 ]
}

# ---------------------------------------------------------------- dispatch
usage() { sed -n '2,25p' "$0" | sed 's/^# \{0,1\}//'; }
[ $# -lt 1 ] && { usage; exit 0; }
cmd="$1"; shift
case "$cmd" in
  du)         cmd_du "$@" ;;
  list)       cmd_list "$@" ;;
  push)       cmd_push "$@" ;;
  pull)       cmd_pull "$@" ;;
  verify)     cmd_verify "$@" ;;
  rm)         cmd_rm "$@" ;;
  vault-rm)   cmd_vault_rm "$@" ;;
  migrate)    cmd_migrate "$@" ;;
  git-remote) cmd_git_remote "$@" ;;
  doctor)     cmd_doctor "$@" ;;
  selftest)   cmd_selftest "$@" ;;
  -h|--help|help) usage ;;
  *) usage; die "unknown command: $cmd" ;;
esac
