#!/bin/sh
set -eu

setup_can() {
    name="$1"
    bitrate="$2"
    usb_port="$3"

    if ip link show "$name" >/dev/null 2>&1; then
        iface="$name"
    else
        iface=""
        for candidate in $(ip -br link show type can | awk '{print $1}'); do
            bus_info="$(ethtool -i "$candidate" 2>/dev/null | awk '/bus-info:/ {print $2}')"
            port_path="${bus_info#*-}"
            if [ "$bus_info" = "$usb_port" ] || [ "$port_path" = "$usb_port" ]; then
                if [ -n "$iface" ]; then
                    echo "ERROR: multiple CAN interfaces match USB port $usb_port"
                    echo "Current CAN interfaces:"
                    ip -details link show type can || true
                    exit 1
                fi
                iface="$candidate"
            fi
        done

        if [ -z "$iface" ]; then
            echo "ERROR: cannot find CAN interface for $name at USB port $usb_port"
            echo "Current CAN interfaces:"
            ip -details link show type can || true
            exit 1
        fi

        sudo ip link set "$iface" down
        sudo ip link set "$iface" name "$name"
        iface="$name"
    fi

    sudo ip link set "$iface" down
    sudo ip link set "$iface" type can bitrate "$bitrate"
    sudo ip link set "$iface" up

    echo "OK: $name -> USB port $usb_port, bitrate $bitrate"
}

sudo modprobe can can_raw

if ! command -v ethtool >/dev/null 2>&1; then
    echo "ERROR: ethtool is not installed. Run: sudo apt install ethtool"
    exit 1
fi

if ! command -v candump >/dev/null 2>&1; then
    echo "ERROR: can-utils is not installed. Run: sudo apt install can-utils"
    exit 1
fi

# Current USB topology on this robot. The root bus number may change
# between boots, so match only the stable port path after the "-".
#   parentdev *-7:1.0     -> waist
#   parentdev *-8:1.0     -> car/base
#   parentdev *-5.3:1.0   -> left NERO arm
#   parentdev *-5.4:1.0   -> right NERO arm
#   parentdev *-5.1.2:1.0 -> hand1
#   parentdev *-5.1.1:1.0 -> hand2
#
# If the USB wiring changes, update the addresses below after checking:
#   ip -details link show type can

setup_can can_waist  1000000 "7:1.0"
setup_can can_car     500000 "8:1.0"
setup_can can_piper1 1000000 "5.3:1.0"
setup_can can_piper2 1000000 "5.4:1.0"
setup_can can_hand1  1000000 "5.1.2:1.0"
setup_can can_hand2  1000000 "5.1.1:1.0"

echo
echo "Final CAN state:"
ip -details link show type can
