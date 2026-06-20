#!/bin/bash

if [ "$(id -u)" != "0" ]; then
    SUDO="sudo"
else
    SUDO=""
fi

# Function to reset a CAN interface
reset_can_interface() {
    local iface=$1
    echo "Resetting CAN interface: $iface"
    $SUDO ip link set "$iface" down
    # restart-ms 100 lets the kernel auto-recover the controller from a bus-off
    # state (e.g. transient bus errors / brief disconnects) instead of leaving
    # the interface DOWN until the next manual reset.
    $SUDO ip link set "$iface" up type can bitrate 1000000 restart-ms 100
}

# Get all CAN interfaces
can_interfaces=$(ip link show | grep -oP '(?<=: )(can\w+)')

# Check if any CAN interfaces were found
if [[ -z "$can_interfaces" ]]; then
    echo "No CAN interfaces found."
    exit 1
fi

# Reset each CAN interface
echo "Detected CAN interfaces: $can_interfaces"
for iface in $can_interfaces; do
    reset_can_interface "$iface"
done

echo "All CAN interfaces have been reset with bitrate 1000000 and restart-ms 100."
