#!/usr/bin/env bash

# RTC/Teleop runner 共用的窄接口。调用方必须先启用 strict mode。

Require_Readable_File() {
    local description="$1"
    local file_path="$2"

    [[ -r "$file_path" ]] || {
        echo "错误：${description}不存在或不可读：$file_path" >&2
        return 1
    }
}

Read_Unique_Yaml_Scalar() {
    local key="$1"
    local config_file="$2"
    local -a values=()

    mapfile -t values < <(
        awk -v key="$key" '
            {
                line = $0
                sub(/[[:space:]]*#.*/, "", line)
                if (line ~ "^[[:space:]]*" key "[[:space:]]*:") {
                    sub("^[[:space:]]*" key "[[:space:]]*:[[:space:]]*", "", line)
                    sub(/[[:space:]]+$/, "", line)
                    if (line ~ /^\".*\"$/ || line ~ /^\047.*\047$/) {
                        line = substr(line, 2, length(line) - 2)
                    }
                    print line
                }
            }
        ' "$config_file"
    )

    if [[ "${#values[@]}" -ne 1 || -z "${values[0]}" ]]; then
        echo "错误：配置必须且只能包含一个非空 ${key}：$config_file" >&2
        return 1
    fi
    printf '%s\n' "${values[0]}"
}

Read_Yaml_Inline_List() {
    local key="$1"
    local config_file="$2"
    local raw_value

    raw_value="$(Read_Unique_Yaml_Scalar "$key" "$config_file")"
    if [[ "$raw_value" != \[*\] ]]; then
        echo "错误：${key} 必须使用单行 YAML 数组：$config_file" >&2
        return 1
    fi

    raw_value="${raw_value:1:${#raw_value}-2}"
    [[ -n "${raw_value//[[:space:]]/}" ]] || return 0
    awk -v value="$raw_value" 'BEGIN {
        count = split(value, items, ",")
        for (item_index = 1; item_index <= count; ++item_index) {
            item = items[item_index]
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", item)
            if (item ~ /^\".*\"$/ || item ~ /^\047.*\047$/) {
                item = substr(item, 2, length(item) - 2)
            }
            if (item == "") {
                exit 1
            }
            print item
        }
    }' || {
        echo "错误：${key} 包含空元素：$config_file" >&2
        return 1
    }
}

Validate_Teleop_Grant_Keys() {
    local config_file="$1"
    local -a key_ids=()
    local -a public_keys=()

    mapfile -t key_ids < <(Read_Yaml_Inline_List grant_key_ids "$config_file")
    mapfile -t public_keys < <(Read_Yaml_Inline_List grant_public_keys "$config_file")
    if [[ "${#key_ids[@]}" -eq 0 || "${#key_ids[@]}" -ne "${#public_keys[@]}" ]]; then
        echo "错误：非 disabled Teleop 必须配置数量相同且非空的 grant_key_ids/grant_public_keys。" >&2
        return 1
    fi
}

Require_Production_Gate() {
    local gate_name="$1"
    local gate_value="${!gate_name:-0}"

    if [[ "$gate_value" != "1" ]]; then
        echo "错误：生产能力需要在 /etc/astrabot/rtc-teleop.env 中显式设置 ${gate_name}=1。" >&2
        return 1
    fi
}

Load_Astrabot_Ros_Environment() {
    local startup_environment_file="$1"
    local ros_config_file="$2"
    local ros_setup_file="$3"

    Require_Readable_File "robot_start 设备环境" "$startup_environment_file"
    Require_Readable_File "robot_start ROS 配置" "$ros_config_file"
    Require_Readable_File "Astrabot ROS 环境" "$ros_setup_file"

    set +u
    # 与 robot_start 其他模块保持相同顺序：设备环境提供 Domain ID，ROS 配置提供 RMW/DDS，最后叠加 overlay。
    # shellcheck disable=SC1090
    source "$startup_environment_file"
    # shellcheck disable=SC1090
    source "$ros_config_file"
    # shellcheck disable=SC1090
    source "$ros_setup_file"
    set -u
    command -v ros2 >/dev/null 2>&1 || {
        echo "错误：Astrabot ROS 环境中找不到 ros2。" >&2
        return 1
    }
}

Require_Ros_Service() {
    local service_name="$1"

    if ! timeout 5 ros2 service type "$service_name" >/dev/null 2>&1; then
        echo "错误：生产 Teleop 依赖的 ROS service 不可用：$service_name" >&2
        return 1
    fi
}
