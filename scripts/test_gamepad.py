#!/usr/bin/env python3
"""
Interactive gamepad test script.

Displays all raw axis values, button states, and computed deltas in real-time.
Supports both pygame (Linux) and HIDAPI (macOS) backends.

Usage:
    python i2rt/scripts/test_gamepad.py [--hid]

    --hid   Force HIDAPI backend (default on macOS, optional on Linux)
"""

import argparse
import os
import sys
import time


def clear_screen():
    os.system("cls" if os.name == "nt" else "clear")


def test_pygame_gamepad():
    """Test gamepad using pygame backend (all raw axes + buttons)."""
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    import pygame

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No gamepad detected via pygame.")
        print("Try running with --hid for HIDAPI backend (required on macOS).")
        return

    joy = pygame.joystick.Joystick(0)
    joy.init()

    num_axes = joy.get_numaxes()
    num_buttons = joy.get_numbuttons()
    num_hats = joy.get_numhats()

    print(f"Connected: {joy.get_name()}")
    print(f"  Axes: {num_axes}  Buttons: {num_buttons}  Hats: {num_hats}")
    print("Press Ctrl+C to exit.\n")
    time.sleep(1)

    try:
        while True:
            pygame.event.pump()

            lines = []
            lines.append(f"=== Gamepad: {joy.get_name()} (pygame) ===\n")

            lines.append("Axes:")
            for i in range(num_axes):
                val = joy.get_axis(i)
                bar_len = int(abs(val) * 20)
                if val >= 0:
                    bar = " " * 20 + "|" + "#" * bar_len + " " * (20 - bar_len)
                else:
                    bar = " " * (20 - bar_len) + "#" * bar_len + "|" + " " * 20
                lines.append(f"  Axis {i:2d}: {val:+.4f}  [{bar}]")

            lines.append("\nButtons:")
            btn_line = "  "
            for i in range(num_buttons):
                state = joy.get_button(i)
                btn_line += f"B{i}:{'ON ' if state else 'off'}  "
                if (i + 1) % 8 == 0:
                    lines.append(btn_line)
                    btn_line = "  "
            if btn_line.strip():
                lines.append(btn_line)

            if num_hats > 0:
                lines.append("\nD-Pad / Hats:")
                for i in range(num_hats):
                    hat = joy.get_hat(i)
                    lines.append(f"  Hat {i}: {hat}")

            # Show the computed deltas that flow_base would use
            x = joy.get_axis(1)
            y = joy.get_axis(0)
            th = joy.get_axis(2) if num_axes > 2 else 0.0
            deadzone = 0.05
            dx = -x if abs(x) >= deadzone else 0.0
            dy = y if abs(y) >= deadzone else 0.0
            dth = th if abs(th) >= deadzone else 0.0

            lines.append(f"\nComputed user_cmd (i2rt):  x={dx:+.4f}  y={dy:+.4f}  th={dth:+.4f}")

            # Also show lerobot-style deltas (axes 0,1,3)
            lx = joy.get_axis(0)
            ly = joy.get_axis(1)
            rz = joy.get_axis(3) if num_axes > 3 else 0.0
            ldx = -ly if abs(ly) >= 0.1 else 0.0
            ldy = -lx if abs(lx) >= 0.1 else 0.0
            ldz = -rz if abs(rz) >= 0.1 else 0.0
            lines.append(f"Computed deltas (lerobot): dx={ldx:+.4f}  dy={ldy:+.4f}  dz={ldz:+.4f}")

            lines.append("\n[Ctrl+C to exit]")

            clear_screen()
            print("\n".join(lines))
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        joy.quit()
        pygame.quit()
        print("\nGamepad disconnected.")


def test_hid_gamepad():
    """Test gamepad using HIDAPI backend (raw byte-level data)."""
    import hid

    print("Scanning for HID gamepad devices...")
    devices = hid.enumerate()

    gamepad_keywords = ["Logitech", "Xbox", "PS4", "PS5", "Controller", "Gamepad", "pad"]
    found = None
    for dev in devices:
        name = dev.get("product_string", "")
        if any(kw.lower() in name.lower() for kw in gamepad_keywords):
            found = dev
            break

    if not found:
        print("\nNo known gamepad found via HIDAPI.")
        print("Available HID devices:\n")
        for dev in devices:
            name = dev.get("product_string", "")
            vendor = dev.get("manufacturer_string", "")
            vid = dev.get("vendor_id", 0)
            pid = dev.get("product_id", 0)
            if name:
                print(f"  {vendor} {name}  (VID=0x{vid:04x} PID=0x{pid:04x})")
        print("\nIf your gamepad is listed above, add its name to the gamepad_keywords list.")
        return

    print(f"Found: {found['manufacturer_string']} {found['product_string']}")
    print(f"  VID=0x{found['vendor_id']:04x}  PID=0x{found['product_id']:04x}")
    print("Press Ctrl+C to exit.\n")
    time.sleep(1)

    device = hid.device()
    device.open_path(found["path"])
    device.set_nonblocking(1)

    deadzone = 0.1

    try:
        while True:
            # Read multiple times for stable HID reading
            data = None
            for _ in range(10):
                d = device.read(64)
                if d:
                    data = d

            if data is None:
                time.sleep(0.01)
                continue

            lines = []
            lines.append(f"=== Gamepad: {found['product_string']} (HIDAPI) ===\n")

            # Raw bytes
            hex_str = " ".join(f"{b:02x}" for b in data[:16])
            lines.append(f"Raw data (first 16 bytes): {hex_str}")
            if len(data) > 16:
                hex_str2 = " ".join(f"{b:02x}" for b in data[16:32])
                lines.append(f"Raw data (bytes 16-31):    {hex_str2}")

            # Parse axes (Logitech RumblePad 2 layout: bytes 1-4)
            if len(data) >= 8:
                left_y = (data[1] - 128) / 128.0
                left_x = (data[2] - 128) / 128.0
                right_x = (data[3] - 128) / 128.0
                right_y = (data[4] - 128) / 128.0

                lines.append(f"\nJoystick Axes (Logitech layout):")
                for name, val in [("Left X", left_x), ("Left Y", left_y),
                                  ("Right X", right_x), ("Right Y", right_y)]:
                    bar_len = int(abs(val) * 20)
                    if val >= 0:
                        bar = " " * 20 + "|" + "#" * bar_len + " " * (20 - bar_len)
                    else:
                        bar = " " * (20 - bar_len) + "#" * bar_len + "|" + " " * 20
                    lines.append(f"  {name:8s}: {val:+.4f}  [{bar}]")

                # Button byte
                buttons = data[5]
                lines.append(f"\nButtons byte (data[5]): 0b{buttons:08b}  (0x{buttons:02x})")
                lines.append(f"  Y/Triangle (bit 7): {'PRESSED' if buttons & (1 << 7) else '-'}")
                lines.append(f"  X/Square   (bit 5): {'PRESSED' if buttons & (1 << 5) else '-'}")
                lines.append(f"  A/Cross    (bit 4): {'PRESSED' if buttons & (1 << 4) else '-'}")

                # Trigger/shoulder byte
                trigger_byte = data[6]
                lines.append(f"\nTrigger byte (data[6]): 0b{trigger_byte:08b}  (0x{trigger_byte:02x} = {trigger_byte})")
                lines.append(f"  RB (intervention): {'PRESSED' if trigger_byte in [2, 6, 10, 14] else '-'}")
                lines.append(f"  RT (open gripper): {'PRESSED' if trigger_byte in [8, 10, 12] else '-'}")
                lines.append(f"  LT (close gripper): {'PRESSED' if trigger_byte in [4, 6, 12] else '-'}")

                # Computed deltas
                dx = -left_x if abs(left_x) >= deadzone else 0.0
                dy = -left_y if abs(left_y) >= deadzone else 0.0
                dz = -right_y if abs(right_y) >= deadzone else 0.0
                lines.append(f"\nComputed deltas: dx={dx:+.4f}  dy={dy:+.4f}  dz={dz:+.4f}")

            lines.append("\n[Ctrl+C to exit]")

            clear_screen()
            print("\n".join(lines))
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        device.close()
        print("\nGamepad disconnected.")


def main():
    parser = argparse.ArgumentParser(description="Test gamepad inputs")
    parser.add_argument("--hid", action="store_true", help="Use HIDAPI backend (default on macOS)")
    args = parser.parse_args()

    use_hid = args.hid or sys.platform == "darwin"

    if use_hid:
        print("Using HIDAPI backend...")
        test_hid_gamepad()
    else:
        print("Using pygame backend...")
        test_pygame_gamepad()


if __name__ == "__main__":
    main()
