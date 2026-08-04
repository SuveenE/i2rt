# Flow Base Setup Guide

## Important Notes

⚠️ **Software Updates**: The pre-installed software may be outdated. To access the latest features, log into the base and pull the newest i2rt codebase.

⚠️ **Pi Firmware**: Latest pi firmware is available [here](https://drive.google.com/drive/u/3/folders/1BAvdCFFR2lsmHqKH9YQ_lMbPV0TAIKik?dmr=1&ec=wgc-drive-globalnav-goto) under the PI_firmware folder. If your device doesn't have all necessary settings configured, remove the SD card and burn the latest firmware following [this instruction](../../devices/pi_setup.md).

## Getting Started

### Unboxing

Follow the detailed visual documentation provided in this [unboxing guide](https://www.canva.com/design/DAGvHpqzf-Y/C_ESTYVeHzDPKgkTQZTf0w/view?utm_content=DAGvHpqzf-Y&utm_campaign=designshare&utm_medium=link2&utm_source=uniquelinks&utlId=h74da76f842). Ensure the battery and charging port are connected correctly.

### Initial Setup

1. Install the battery and turn on the base
2. The screen will light up and the Raspberry Pi will begin booting
3. Verify the **E-stop** is **not pressed**
4. Ensure the **CAN bus selection switch** is in the **UP position**

<p align="center">
  <img src="assets/flow_base_panel.jpg" alt="Flow Base Control Panel" width="50%">
</p>

⚠️ **Note**: The small screen firmware may cause slower Pi boot times, but you can SSH into the system quickly once it's ready.

### Quick Start

1. Double-click the **FlowBase** icon on the desktop and run it in terminal
2. Turn on the remote to control the base
3. If the remote is unresponsive, toggle it off and on to wake it from sleep mode

## System Access

### Pi Login Credentials
- **Username**: `i2rt`
- **Password**: `root`

### SSH Access

**Option 1: Wireless Connection**
Connect the Pi to your local network via Wi-Fi (keyboard required for password entry).

**Option 2: Wired Connection**
The exposed RJ45 network interface is preconfigured with static IP `172.6.2.20`.

1. Connect your dev machine to the wired port with an ethernet cable
2. Configure your dev machine's network interface to use `172.6.2.*` IP range
3. SSH using:
   ```bash
   ssh i2rt@172.6.2.20 -J $USER_NAME@$YOUR_DEV_MACHINE_IP
   ```

## Remote Control

<p align="center">
  <img src="assets/remote.jpg" alt="Flow Base Remote" width="50%">
</p>

### Control Layout

- **Left joystick**: Translation (XY movement)
- **Right joystick X-axis**: Rotation
- **Right joystick Y-axis**: Linear rail lift (up/down) - only available when linear rail is installed
- **Left1**: Reset odometry
- **Mode**: Switch between local and global coordinate modes
- **Left2**: Override API commands (safety feature)

### Important Notes
- The base has motion control limits with maximum acceleration constraints
- When you release the joystick (sending 0 command), the base won't stop immediately due to physics
- Always ensure the remote is awake when running API experiments - Left2 can override unexpected code behavior
- Speed and acceleration settings can be adjusted in [flow_base_controller](flow_base_controller.py#L500-L501)

⚠️ **Warning**: Setting overly aggressive speed or acceleration parameters can cause system instability.

## Coordinate Systems

### Local vs Global Mode

⚠️ **Odometry Warning**: Wheel odometry is prone to error accumulation and can be inaccurate. For mobile manipulation requiring precise odometry, integrate visual odometry sensors like RealSense T265 or ZED Camera.

- **Global mode**: Similar to drone headless mode, but wheel odometry errors accumulate
- **Local mode**: Relative to current base orientation
- Press **Mode** button to switch between coordinate systems
- Press **Left1** to reset odometry
- Base screen displays current command: `frame: global cmd: 0.0 0.0 0.0`

## API Control

### Network Setup
1. Connect base to Wi-Fi or use wired connection
2. Base IP address: `172.6.2.20`
3. Verify connectivity: `ping 172.6.2.20`

### Basic Commands

**Read Odometry:**
```python
python i2rt/flow_base/flow_base_client.py --command get_odometry --host 172.6.2.20
```

**Output:**
```bash
[Client] Connecting to 172.6.2.20:11323
[Client] Connection established
{'translation': array([-6.59153544e-07, -3.79215432e-04]), 'rotation': array(-0.00022068)}
```

**Reset Odometry:**
```python
python i2rt/flow_base/flow_base_client.py --command reset_odometry --host 172.6.2.20
```

**Test Movement** ⚠️ **Base will move**:
```python
python i2rt/flow_base/flow_base_client.py --command test_command --host 172.6.2.20
```

**Test Linear Rail** ⚠️ **Linear rail will move**:
```bash
python i2rt/flow_base/flow_base_client.py --command test_linear_rail --host 172.6.2.20
```

**Get Linear Rail State**:
```bash
python i2rt/flow_base/flow_base_client.py --command get_linear_rail_state --host 172.6.2.20
```

### Linear Rail API (if equipped)

If your FlowBase has a linear rail lift module installed, you can control it via API:

**Available Methods:**
- `get_linear_rail_state()` - Get position, velocity, limit switch states
- `set_linear_rail_velocity(velocity)` - Set velocity in rad/s
- `set_target_velocity([x, y, theta, rail_vel], frame)` - Combined base + rail control (4D)

Initialize with `FlowBaseClient(host="172.6.2.20", with_linear_rail=True)` to enable linear rail support.

**Important Notes:**
- Linear rail automatically homes to lower limit on initialization
- Linear rail has limit switches that prevent movement beyond safe range
- Velocity commands timeout after 0.25s of inactivity (safety feature)
- Brake is automatically managed by the system (released on init, engaged on shutdown)
- To stop the rail, set velocity to 0.0 instead of controlling brake directly

### Safety Features
- API command timeout prevents runaway behavior (base: 0.25s, linear rail: 0.25s)
- FlowBaseClient automatically maintains command heartbeat
- Base and linear rail stop automatically when client disconnects
- Use remote Left2 to override API commands in emergencies
- Use remote Left1 to clear odometry during testing
- Linear rail limit switches provide hardware safety stops

## External Control

To control the base without the built-in Raspberry Pi:

1. Connect your external CAN device to the CAN external connector
2. Set the CAN selector switch to the **DOWN position**
3. Clone the i2rt repository on your external computer
4. Control the base directly through your external system

### Linear Rail on x86 / non-Pi hosts (USB-GPIO converter)

On the built-in Raspberry Pi the linear rail's brake and limit switches use the Pi's native GPIO (`RPi.GPIO`) — no extra setup. On an x86 / non-Pi host they are driven through a **bestep USB-to-16-channel GPIO converter** (hardware id `ZT-DPI/SY`) on a serial port. The backend is auto-selected from `platform.machine()`, so the control code is identical on both platforms.

- `--device` is required when the linear rail is enabled on an x86 / non-Pi host (and `--gpio-host` is not set); on the Raspberry Pi it is not needed (native GPIO) and is ignored, e.g.

  ```bash
  python i2rt/flow_base/flow_base_controller.py --device /dev/ttyUSB0
  ```

  (The `I2RT_USB_GPIO_PORT` env var also works for programmatic use; the flag wins.)
- Converter channel wiring: **channel 1 = upper limit switch, channel 2 = lower limit switch, channel 3 = brake**.
- Requires `pyserial` (installed with the package).

Alternatively, keep GPIO on the Pi and drive it remotely: start the GPIO satellite on the Pi (`python i2rt/flow_base/gpio_satellite_server.py --port 8765`) and pass `--gpio-host <RPI_IP>:8765` on the control host.

### Linear Rail over a direct USB serial link (Pi GPIO, no network / SSH)

You can also keep the rail's brake + limit switches on the Pi's native GPIO but reach them over a **direct USB cable** instead of the network. The Pi 5 exposes a **USB CDC-ACM serial gadget** on its USB-C port; you SSH into the Pi to start the serial satellite (just like the network satellite, but over the USB link instead of a network socket).

```text
Pi native GPIO (BCM 12 brake / 5 upper / 6 lower)
   |
   +-- LocalGPIOBackend  <--  gpio_serial_satellite_server.py  (started over SSH on the Pi)
                                        |
                                        |  /dev/ttyGS0  (CDC-ACM gadget)
                                        v
                                   [ USB-C cable ]
                                        |
                                        v  /dev/ttyACM0  (on the control host)
                            SerialSatelliteBackend  ->  flow_base_controller.py
```

One-time gadget setup on the Pi (Pi 5; run once, then reboot):

```bash
sudo bash scripts/setup_pi_usb_gpio_gadget.sh
sudo reboot
```

This adds `dtoverlay=dwc2,dr_mode=peripheral` to `config.txt` and loads `dwc2`/`g_serial` at boot (creating `/dev/ttyGS0`). Use the Pi 5's **USB-C port** (the only OTG/peripheral-capable port).

Start the satellite on the Pi over SSH:

```bash
ssh i2rt@<PI_HOST>
cd i2rt && python i2rt/flow_base/gpio_serial_satellite_server.py --port /dev/ttyGS0
```

On the control host, verify the link, then run the controller:

```bash
python i2rt/scripts/test_gpio_serial.py --device /dev/ttyACM0
python i2rt/flow_base/flow_base_controller.py --gpio-serial /dev/ttyACM0
```

The serial protocol is newline-delimited ASCII (`INIT_BRAKE` / `BRAKE 1|0` / `INIT_LIMITS` / `GET` -> `L <upper> <lower>` / `CLEANUP`); the host polls `GET` at ~100 Hz. Requires `pyserial` on both the Pi and the control host.

Notes / caveats:
- On the Pi 5, `dr_mode=peripheral` turns the USB-C port into a power+gadget-data port only. Power the Pi from the base supply (or the 5V GPIO header) so the USB-C port is free for the PC link.
- The serial gadget (`g_serial`) and the Ethernet gadget (`g_ether`) are mutually exclusive on this simple gadget, so SSH-over-USB-ethernet is not available while the serial gadget is active; keep wifi / wired LAN for Pi admin.
- Precedence on the control host: `--gpio-serial` > `--gpio-host` > native Pi GPIO > USB-to-GPIO converter.

Wiring (the `BCM N` are the controller's logical pins, mapped to converter channels by `USB_GPIO_CHANNEL_MAP`):

```text
x86 host --[USB 115200 8N1]--> bestep USB-to-16ch GPIO converter (ZT-DPI/SY),
                               enumerates as /dev/ttyUSB0
                               |
                               +-- 3.3V --> upper/lower limit switches (common)
                               +-- ch1  --> upper limit switch   (BCM 5)
                               +-- ch2  --> lower limit switch   (BCM 6)
                               +-- ch3  --> brake control signal (BCM 12)
                               +-- GND  --> brake driver GND
```

## Troubleshooting

- **Remote unresponsive**: Toggle remote off and on to wake from sleep
- **Slow boot**: Screen firmware causes delays, but SSH access is available quickly
- **Inaccurate odometry**: Expected with wheel-based systems, especially during aggressive movements
- **Linear rail not homing**: Check GPIO connections and limit switches. Ensure brake is released
- **Linear rail stuck at limit**: Check limit switch state. Use `get_linear_rail_state()` to verify switch 
- **Assertion error: failed to communicate with motor 3**: Verify the E-stop is not pressed and the CAN multiplexer switch is in the correct position. For the internal Pi, the switch must be UP; for an external computer, the switch must be DOWN.
