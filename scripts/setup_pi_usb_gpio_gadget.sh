#!/usr/bin/env bash
#
# Configure a Raspberry Pi 5 USB-C port as a CDC-ACM serial gadget:
#
#   Pi /dev/ttyGS0  <--[USB-C cable]-->  host /dev/ttyACM0
#
# Run on the Pi with sudo, then reboot.

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "This script configures boot files; please run with sudo." >&2
  exit 1
fi

CONFIG_TXT="${CONFIG_TXT:-/boot/firmware/config.txt}"
[[ -f "$CONFIG_TXT" ]] || CONFIG_TXT=/boot/config.txt
GADGET_DEVICE="${GADGET_DEVICE:-/dev/ttyGS0}"

echo "Using CONFIG_TXT=${CONFIG_TXT}"

if grep -qE '^\s*dtoverlay=dwc2,dr_mode=peripheral' "$CONFIG_TXT"; then
  echo "[config.txt] dwc2 peripheral overlay already present."
else
  echo "[config.txt] adding dwc2 peripheral overlay under [all]."
  {
    echo ""
    echo "[all]"
    echo "# Linear-rail GPIO-over-USB serial gadget."
    echo "dtoverlay=dwc2,dr_mode=peripheral"
  } >> "$CONFIG_TXT"
fi

MODULES_CONF=/etc/modules-load.d/gpio-usb-gadget.conf
echo "[modules] writing ${MODULES_CONF}."
cat > "$MODULES_CONF" <<'EOF'
dwc2
g_serial
EOF

# Prevent a login console from competing with the satellite for ttyGS0.
if systemctl list-unit-files 2>/dev/null | grep -q '^serial-getty@'; then
  systemctl mask "serial-getty@$(basename "$GADGET_DEVICE").service" \
    2>/dev/null || true
fi

echo ""
echo "Done. Reboot the Pi:"
echo "    sudo reboot"
echo ""
echo "Then start the satellite:"
echo "    python i2rt/flow_base/gpio_serial_satellite_server.py --port ${GADGET_DEVICE}"
