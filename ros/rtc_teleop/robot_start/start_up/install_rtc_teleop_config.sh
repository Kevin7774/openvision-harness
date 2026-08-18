#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="${ASTRABOT_CONFIG_DIR:-/etc/astrabot}"
CONFIG_OWNER="${ASTRABOT_CONFIG_OWNER:-root}"
CONFIG_GROUP="${ASTRABOT_CONFIG_GROUP:-astrabot}"

RTC_TEMPLATE="${ASTRABOT_RTC_CONFIG_TEMPLATE:-${SCRIPT_DIR}/run_script/thor/supplement/config/rtc/rtc.yaml.example}"
TELEOP_TEMPLATE="${ASTRABOT_TELEOP_CONFIG_TEMPLATE:-${SCRIPT_DIR}/run_script/thor/supplement/config/teleop/teleop.yaml.example}"
GATE_TEMPLATE="${ASTRABOT_RTC_TELEOP_GATE_TEMPLATE:-${SCRIPT_DIR}/run_script/thor/supplement/config/rtc/rtc-teleop.env.example}"

# 安装后的脚本位于 function/，模板位于相邻 config/；源码目录仍优先使用 run_script 下的模板。
if [[ ! -f "$RTC_TEMPLATE" && -f "${SCRIPT_DIR}/../config/rtc/rtc.yaml.example" ]]; then
    RTC_TEMPLATE="${SCRIPT_DIR}/../config/rtc/rtc.yaml.example"
fi
if [[ ! -f "$TELEOP_TEMPLATE" && -f "${SCRIPT_DIR}/../config/teleop/teleop.yaml.example" ]]; then
    TELEOP_TEMPLATE="${SCRIPT_DIR}/../config/teleop/teleop.yaml.example"
fi
if [[ ! -f "$GATE_TEMPLATE" && -f "${SCRIPT_DIR}/../config/rtc/rtc-teleop.env.example" ]]; then
    GATE_TEMPLATE="${SCRIPT_DIR}/../config/rtc/rtc-teleop.env.example"
fi

if [[ $# -ne 0 ]]; then
    echo "用法：$0" >&2
    exit 2
fi

if [[ -z "$CONFIG_DIR" || "$CONFIG_DIR" != /* || "$CONFIG_DIR" == "/" ]]; then
    echo "错误：ASTRABOT_CONFIG_DIR 必须是非根绝对路径：${CONFIG_DIR:-<empty>}" >&2
    exit 2
fi

Run_Privileged() {
    if [[ "${EUID}" -eq 0 ]]; then
        "$@"
        return
    fi

    command -v sudo >/dev/null 2>&1 || {
        echo "错误：当前用户不是 root，且找不到 sudo。" >&2
        return 1
    }
    sudo "$@"
}

Install_Config_If_Missing() {
    local template_file="$1"
    local target_file="$2"

    [[ -f "$template_file" ]] || {
        echo "错误：找不到配置模板：$template_file" >&2
        return 1
    }
    if [[ -L "$target_file" ]]; then
        echo "错误：拒绝覆盖符号链接配置：$target_file" >&2
        return 1
    fi
    if [[ -e "$target_file" && ! -f "$target_file" ]]; then
        echo "错误：配置目标不是普通文件：$target_file" >&2
        return 1
    fi

    if [[ -f "$target_file" ]]; then
        echo "保留已有实时通信配置：$target_file"
    else
        Run_Privileged install -o "$CONFIG_OWNER" -g "$CONFIG_GROUP" -m 0640 \
            "$template_file" "$target_file"
        echo "已安装安全默认配置：$target_file"
    fi

    Run_Privileged chown "$CONFIG_OWNER:$CONFIG_GROUP" "$target_file"
    Run_Privileged chmod 0640 "$target_file"
}

if [[ -L "$CONFIG_DIR" ]]; then
    echo "错误：拒绝使用符号链接配置目录：$CONFIG_DIR" >&2
    exit 1
fi
if [[ -e "$CONFIG_DIR" && ! -d "$CONFIG_DIR" ]]; then
    echo "错误：配置目录目标不是目录：$CONFIG_DIR" >&2
    exit 1
fi
if [[ -d "$CONFIG_DIR" ]]; then
    # /etc/astrabot 还承载 board.yaml 等既有配置，不修改共享目录的 owner/mode。
    echo "保留已有配置目录属性：$CONFIG_DIR"
else
    Run_Privileged install -d -o "$CONFIG_OWNER" -g "$CONFIG_GROUP" -m 0750 "$CONFIG_DIR"
fi
Install_Config_If_Missing "$RTC_TEMPLATE" "$CONFIG_DIR/rtc.yaml"
Install_Config_If_Missing "$TELEOP_TEMPLATE" "$CONFIG_DIR/teleop.yaml"
Install_Config_If_Missing "$GATE_TEMPLATE" "$CONFIG_DIR/rtc-teleop.env"

echo "RTC/Teleop 配置已就绪；生产 Gate 默认关闭，服务仍需显式启动。"
