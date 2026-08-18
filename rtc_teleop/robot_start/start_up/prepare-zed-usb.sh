#!/usr/bin/env bash
set -Eeuo pipefail

readonly ZED_VIDEO_VENDOR_ID="2b03"
readonly ZED_VIDEO_PRODUCT_ID="f880"
readonly ZED_HID_VENDOR_ID="2b03"
readonly ZED_HID_PRODUCT_ID="f881"

read_sysfs_value() {
    local path="$1"
    tr -d '[:space:]' < "$path"
}

find_zed_hid_interfaces() {
    local sysfs_root="$1"
    local device_dir
    local interface_dir

    for device_dir in "$sysfs_root"/bus/usb/devices/*; do
        [[ -d "$device_dir" && -f "$device_dir/idVendor" && -f "$device_dir/idProduct" ]] || continue
        [[ "$(read_sysfs_value "$device_dir/idVendor")" == "$ZED_HID_VENDOR_ID" ]] || continue
        [[ "$(read_sysfs_value "$device_dir/idProduct")" == "$ZED_HID_PRODUCT_ID" ]] || continue

        for interface_dir in "$device_dir":*; do
            [[ -d "$interface_dir" && -f "$interface_dir/bInterfaceClass" ]] || continue
            [[ "$(read_sysfs_value "$interface_dir/bInterfaceClass")" == "03" ]] || continue
            printf '%s\n' "$interface_dir"
        done
    done
}

zed_video_device_present() {
    local sysfs_root="$1"
    local device_dir

    for device_dir in "$sysfs_root"/bus/usb/devices/*; do
        [[ -d "$device_dir" && -f "$device_dir/idVendor" && -f "$device_dir/idProduct" ]] || continue
        if [[ "$(read_sysfs_value "$device_dir/idVendor")" == "$ZED_VIDEO_VENDOR_ID" &&
              "$(read_sysfs_value "$device_dir/idProduct")" == "$ZED_VIDEO_PRODUCT_ID" ]]; then
            return 0
        fi
    done
    return 1
}

bind_usb_interface() {
    local sysfs_root="$1"
    local interface_name="$2"
    printf '%s' "$interface_name" > "$sysfs_root/bus/usb/drivers/usbhid/bind"
}

prepare_zed_hid_interface() {
    local sysfs_root="$1"
    local interface_dir="$2"
    local interface_name
    local driver_name=""

    interface_name="$(basename "$interface_dir")"
    if [[ -L "$interface_dir/driver" ]]; then
        driver_name="$(basename "$(readlink -f "$interface_dir/driver")")"
    fi

    case "$driver_name" in
        usbhid)
            echo "ZED HID interface already ready: $interface_name"
            return 0
            ;;
        "")
            echo "Restoring ZED HID driver binding: $interface_name"
            bind_usb_interface "$sysfs_root" "$interface_name"
            ;;
        usbfs)
            echo "ZED HID interface is already owned by another process: $interface_name" >&2
            return 1
            ;;
        *)
            echo "ZED HID interface uses unexpected driver '$driver_name': $interface_name" >&2
            return 1
            ;;
    esac

    if [[ ! -L "$interface_dir/driver" ||
          "$(basename "$(readlink -f "$interface_dir/driver")")" != "usbhid" ]]; then
        echo "ZED HID driver binding did not become ready: $interface_name" >&2
        return 1
    fi
}

prepare_zed_usb() {
    local sysfs_root="$1"
    local interface_dir
    local interface_count=0

    if ! zed_video_device_present "$sysfs_root"; then
        echo "ZED video interface 2b03:f880 is not present" >&2
        return 1
    fi

    while IFS= read -r interface_dir; do
        [[ -n "$interface_dir" ]] || continue
        interface_count=$((interface_count + 1))
        prepare_zed_hid_interface "$sysfs_root" "$interface_dir"
    done < <(find_zed_hid_interfaces "$sysfs_root")

    if (( interface_count == 0 )); then
        echo "ZED HID interface 2b03:f881 is not present" >&2
        return 1
    fi
}

main() {
    if (( EUID != 0 )); then
        echo "prepare-zed-usb.sh must run as root" >&2
        return 1
    fi
    prepare_zed_usb /sys
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    main "$@"
fi
