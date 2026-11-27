#!/usr/bin/env bash
# Import the newest Robocon training result, retaining a rollback copy.
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="$PROJECT_DIR/.venv-deploy/bin/python"
ROBOT_LAB="$PROJECT_DIR/../robot_lab"
LOGS_ROOT=""
ASSET_ROOT=""
RUN_DIR=""
CHECKPOINT=""
DRY_RUN=false

usage() {
    cat <<'EOF'
用法：
  ./scripts/import_latest_ice_robocon.sh
  ./scripts/import_latest_ice_robocon.sh --dry-run
  ./scripts/import_latest_ice_robocon.sh --run-dir /path/to/training/run

自动查找 robot_lab/logs/rsl_rl 下最新的 Robocon 训练结果，排除 smoke 测试。
按 checkpoint 修改时间选择最新 run，再取该 run 中编号最大的 model_N.pt。
自动备份并替换策略、配置和模型；失败时自动恢复。不会启动仿真。

选项：
  --robot-lab DIR   robot_lab 仓库位置，默认与 rl_sar 同级
  --logs-root DIR   训练日志根目录，默认 robot_lab/logs/rsl_rl
  --asset-root DIR  Robocon 资源目录，默认使用 robot_lab 内的资源
  --run-dir DIR     指定某次训练目录，仍自动取最大编号 checkpoint
  --checkpoint NAME 指定 model_N.pt 文件名，须同时提供 --run-dir
  --dry-run        只显示将要导入的策略，不修改文件
  -h, --help       显示帮助
EOF
}

die() { echo "错误：$*" >&2; exit 1; }
while (( $# )); do
    case "$1" in
        --robot-lab|--logs-root|--asset-root|--run-dir|--checkpoint)
            (( $# >= 2 )) || die "$1 缺少参数"
            case "$1" in
                --robot-lab) ROBOT_LAB="$2" ;;
                --logs-root) LOGS_ROOT="$2" ;;
                --asset-root) ASSET_ROOT="$2" ;;
                --run-dir) RUN_DIR="$2" ;;
                --checkpoint) CHECKPOINT="$2" ;;
            esac
            shift 2 ;;
        --dry-run) DRY_RUN=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) die "未知参数 $1（使用 --help 查看用法）" ;;
    esac
done
[[ -x "$PYTHON" ]] || die "缺少 .venv-deploy，请先运行 bash scripts/setup_ice_robocon.sh"
LOGS_ROOT="${LOGS_ROOT:-$ROBOT_LAB/logs/rsl_rl}"
ASSET_ROOT="${ASSET_ROOT:-$ROBOT_LAB/source/robot_lab/robot_lab/assets/data/Robots/ice/robocon}"

selection="$("$PYTHON" - "$LOGS_ROOT" "$RUN_DIR" "$CHECKPOINT" <<'PY'
from pathlib import Path
import re
import sys
import yaml

class Loader(yaml.SafeLoader):
    pass
for tag in ('python/tuple', 'python/object/apply:builtins.slice'):
    Loader.add_constructor('tag:yaml.org,2002:' + tag, lambda loader, node: loader.construct_sequence(node))

logs, explicit_run, checkpoint = sys.argv[1:]
if checkpoint and not explicit_run:
    sys.exit('--checkpoint 必须与 --run-dir 一起使用')
if checkpoint and not re.fullmatch(r'model_\d+\.pt', checkpoint):
    sys.exit('--checkpoint 必须是 model_N.pt 文件名')
expected = [f'{leg}_{joint}_joint' for leg in ('fr', 'fl', 'rr', 'rl') for joint in ('hip', 'thigh', 'calf')]
expected += [f'{leg}_foot_joint' for leg in ('fr', 'fl', 'rr', 'rl')]

def inspect(run):
    if not (run/'params/env.yaml').is_file() or not (run/'params/agent.yaml').is_file():
        return None
    env = yaml.load((run/'params/env.yaml').read_text(), Loader=Loader)
    if not isinstance(env, dict) or env.get('joint_names') != expected:
        return None
    files = [(int(m.group(1)), p) for p in run.glob('model_*.pt')
             if (m := re.fullmatch(r'model_(\d+)\.pt', p.name)) and p.is_file() and p.stat().st_size]
    if not files:
        return None
    chosen = run/checkpoint if checkpoint else max(files, key=lambda item: item[0])[1]
    if not chosen.is_file() or chosen.stat().st_size == 0:
        sys.exit(f'checkpoint 不存在或为空：{chosen}')
    return max(p.stat().st_mtime_ns for _, p in files), chosen

if explicit_run:
    run = Path(explicit_run).expanduser().resolve()
    result = inspect(run)
    if result is None:
        sys.exit(f'不是有效的 Robocon 训练结果（需要 params 和 checkpoint）：{run}')
    chosen = result[1]
else:
    root = Path(logs).expanduser().resolve()
    if not root.is_dir():
        sys.exit(f'训练日志目录不存在：{root}')
    candidates = []
    for file in sorted(root.glob('*/*/params/env.yaml')):
        run = file.parent.parent
        if 'smoke' in run.name.lower() or 'smoke' in run.parent.name.lower():
            continue
        try:
            result = inspect(run)
        except (OSError, ValueError, yaml.YAMLError) as error:
            print(f'跳过无法读取的训练目录：{run} ({error})', file=sys.stderr)
            continue
        if result:
            candidates.append((result[0], str(run), result[1]))
    if not candidates:
        sys.exit(f'没有找到可用的非 smoke Robocon 策略：{root}')
    _, directory, chosen = max(candidates, key=lambda item: (item[0], item[1]))
    run = Path(directory)
for value in (str(run), chosen.name):
    if '\n' in value or '\r' in value:
        sys.exit('路径不能含换行符')
    print(value)
PY
)"
mapfile -t selected <<< "$selection"
RUN_DIR="${selected[0]}"
CHECKPOINT="${selected[1]}"
echo "训练目录：$RUN_DIR"
echo "即将导入：$CHECKPOINT"
echo "目标策略：$PROJECT_DIR/policy/ice/robocon/robot_lab/policy.pt"
if [[ "$DRY_RUN" == true ]]; then exit 0; fi

[[ -f "$ASSET_ROOT/urdf/robocon.urdf" && -d "$ASSET_ROOT/meshes" ]] || die "Robocon 模型资源不完整：$ASSET_ROOT"
command -v flock >/dev/null || die "缺少 flock 命令"
mkdir -p "$PROJECT_DIR/logs/robocon_policy_backups"
exec 9>"$PROJECT_DIR/logs/robocon_policy_backups/.import.lock"
flock -n 9 || die "另一个策略导入任务正在执行"
BACKUP="$(mktemp -d "$PROJECT_DIR/logs/robocon_policy_backups/$(date +%Y%m%d_%H%M%S)_XXXXXX")"
POLICY_DIR="$PROJECT_DIR/policy/ice/robocon"
MODEL_DIR="$PROJECT_DIR/src/rl_sar_zoo/ice/robocon_description"
HAD_POLICY=false
HAD_MODEL=false
if [[ -d "$POLICY_DIR" ]]; then
    cp -a --reflink=auto "$POLICY_DIR" "$BACKUP/robocon"
    HAD_POLICY=true
fi
if [[ -d "$MODEL_DIR" ]]; then
    cp -a --reflink=auto "$MODEL_DIR" "$BACKUP/robocon_description"
    HAD_MODEL=true
fi
echo "原策略及模型备份：$BACKUP"

rollback() {
    status=$?
    trap - EXIT INT TERM
    if (( status != 0 )); then
        echo "导入失败，恢复导入前的策略和模型……" >&2
        rm -rf -- "$POLICY_DIR" "$MODEL_DIR"
        if [[ "$HAD_POLICY" == true ]]; then cp -a "$BACKUP/robocon" "$POLICY_DIR"; fi
        if [[ "$HAD_MODEL" == true ]]; then cp -a "$BACKUP/robocon_description" "$MODEL_DIR"; fi
        echo "导入日志与备份保留在：$BACKUP" >&2
    fi
    exit "$status"
}
trap rollback EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

"$PYTHON" "$PROJECT_DIR/scripts/import_ice_robocon.py" \
    --run-dir "$RUN_DIR" --asset-root "$ASSET_ROOT" --checkpoint "$CHECKPOINT" --force \
    2>&1 | tee "$BACKUP/import.log"

trap - EXIT INT TERM
echo "策略替换完成，来源记录已更新：policy/ice/robocon/robot_lab/source.yaml"
echo "请重启仿真以加载新策略：$PROJECT_DIR/scripts/run_ice_robocon.sh scene --auto-policy"
