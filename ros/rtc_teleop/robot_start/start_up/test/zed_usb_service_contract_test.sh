#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
START_UP_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"

assert_contains() {
    local file="$1"
    local expected="$2"
    grep -Fq -- "$expected" "$file" || {
        echo "contract missing: $file does not contain $expected" >&2
        return 1
    }
}

bash -n "$START_UP_DIR/prepare-zed-usb.sh"
bash -n "$START_UP_DIR/generate_service.sh"

test_root="$(mktemp -d /tmp/astrabot-zed-usb-test.XXXXXX)"
trap 'rm -rf -- "$test_root"' EXIT

sysfs_root="$test_root/sys"
video_device="$sysfs_root/bus/usb/devices/2-1"
hid_device="$sysfs_root/bus/usb/devices/1-2.2"
hid_interface="$sysfs_root/bus/usb/devices/1-2.2:1.0"
usbhid_driver="$sysfs_root/bus/usb/drivers/usbhid"
mkdir -p "$video_device" "$hid_device" "$hid_interface" "$usbhid_driver"
printf '%s\n' 2b03 > "$video_device/idVendor"
printf '%s\n' f880 > "$video_device/idProduct"
printf '%s\n' 2b03 > "$hid_device/idVendor"
printf '%s\n' f881 > "$hid_device/idProduct"
printf '%s\n' 03 > "$hid_interface/bInterfaceClass"
: > "$usbhid_driver/bind"

# shellcheck source=/dev/null
source "$START_UP_DIR/prepare-zed-usb.sh"

bind_usb_interface() {
    local unused_sysfs_root="$1"
    local interface_name="$2"
    [[ -n "$unused_sysfs_root" ]]
    ln -s "$usbhid_driver" "$sysfs_root/bus/usb/devices/$interface_name/driver"
}

prepare_zed_usb "$sysfs_root"
[[ "$(basename "$(readlink -f "$hid_interface/driver")")" == "usbhid" ]]

rm -f "$hid_interface/driver"
usbfs_driver="$sysfs_root/bus/usb/drivers/usbfs"
mkdir -p "$usbfs_driver"
ln -s "$usbfs_driver" "$hid_interface/driver"
if prepare_zed_hid_interface "$sysfs_root" "$hid_interface"; then
    echo "contract violation: busy ZED HID interface must be rejected" >&2
    exit 1
fi
rm -f "$hid_interface/driver"

environment_file="$test_root/environment.sh"
service_file="$test_root/Astrabot_ZED.service"
printf '%s\n' 'THE_USER=astrabot' > "$environment_file"

# shellcheck source=/dev/null
source "$START_UP_DIR/generate_service.sh"
Generate_Service "$test_root" Astrabot_ZED "$environment_file" /bin/true "$service_file" 1 thor

assert_contains "$service_file" "ExecStartPre=+/usr/local/bin/prepare-zed-usb.sh"
assert_contains "$service_file" "KillSignal=SIGINT"
assert_contains "$service_file" "KillMode=control-group"
assert_contains "$service_file" "TimeoutStopSec=20"
assert_contains "$START_UP_DIR/install.sh" 'cp prepare-zed-usb.sh "${START_UP_DIR}/${RUN_DIR}"'
assert_contains "$START_UP_DIR/install.sh" '/usr/local/bin/prepare-zed-usb.sh'
assert_contains "$START_UP_DIR/reload_auto_start_script.sh" 'prepare-zed-usb.sh'

zed_start_script="$START_UP_DIR/run_script/thor/Astrabot_ZED.start_script"
zed_override="$START_UP_DIR/run_script/thor/supplement/config/zed/zed2i-thor.yaml"
rtc_config="$START_UP_DIR/run_script/thor/supplement/config/rtc/rtc.yaml.example"
assert_contains "$zed_start_script" 'ros_params_override_path:=/opt/ros/start_up/config/zed/zed2i-thor.yaml'
assert_contains "$zed_override" 'grab_resolution: HD720'
assert_contains "$zed_override" 'grab_frame_rate: 15'
assert_contains "$zed_override" 'self_calib: false'
assert_contains "$zed_override" 'publish_left_right: true'
assert_contains "$rtc_config" 'image_topics: /zed/zed_node/left/color/rect/image,/zed/zed_node/right/color/rect/image'
assert_contains "$rtc_config" 'camera_info_topics: /zed/zed_node/left/color/rect/camera_info,/zed/zed_node/right/color/rect/camera_info'

if command -v systemd-analyze >/dev/null 2>&1 &&
   [[ -x /usr/local/bin/prepare-zed-usb.sh ]]; then
    systemd-analyze verify "$service_file"
fi
