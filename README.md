# RasPike Bridge PS5

PTY and Unix domain socket bridge for RasPike, plus a PS5/DualSense controller
client.

## Layout

```text
libraspike
  |
  v
/tmp/raspike-tty
  |
  v
raspike_bridge.py -> /dev/USB_SPIKE -> RasPike firmware
       ^
       |
ps5_raspike_control.py
ps5_controller/
```

The bridge keeps the existing `libraspike` serial protocol intact while allowing
another process, such as the PS5 controller client, to send raw RasPike protocol
packets through a Unix domain socket.

## Requirements

- Linux
- Python 3.10+
- `evdev`
- Read access to `/dev/input/event*` for the controller
- Access to the SPIKE serial device, for example `/dev/USB_SPIKE`

Install Python dependencies:

```bash
python3 -m pip install -r requirements.txt
```

If controller access fails, add the user to the `input` group and log in again:

```bash
sudo usermod -aG input "$USER"
```

## Run

Start the bridge:

```bash
python3 raspike_bridge.py \
  --serial /dev/USB_SPIKE \
  --pty-link "$HOME/raspike-tty" \
  --unix-socket /tmp/raspike.sock \
  --pty-priority-ms 200 \
  -v
```

Point existing `libraspike` code at `$HOME/raspike-tty`.

When `libraspike` is connected through the PTY, do not pass
`--spike-handshake`; `libraspike` performs the SPIKE `AE CE` startup handshake
itself. Use `--spike-handshake` only when running socket clients without
`libraspike`.

Start PS5 control:

```bash
python3 ps5_raspike_control.py \
  --socket /tmp/raspike.sock \
  --left-port B \
  --right-port A \
  --arm-port C \
  --force-port D \
  --max-power 60 \
  --config ./ps5_controller.yaml \
  -v
```

If no arm motor is connected, add `--no-arm`. This skips the optional C-port
motor setup and disables arm controls.

Startup device setup is intentionally paced to avoid overrunning the SPIKE port
device initialization. Tune it with `--init-delay-sec` and `--init-retries` if
the third or later device reports a transient `ack=0`. Use `--init-order
arm-first` to configure the optional arm motor before the drive motors.

By default, the PS5 control process also exposes:

```text
UDP telemetry: 127.0.0.1:8765
TCP Web control input: 127.0.0.1:8766
```

Telemetry is one-way UDP JSON for motor, IMU, battery, and control state.
Web control input is TCP JSON lines so browser keyboard commands do not share
the best-effort telemetry path. Use `--no-telemetry` or `--no-web-control` to
disable either side.

When running the WebUI on a separate PC over a trusted LAN, send telemetry to
the PC and expose Web control on the RasPi LAN interface:

```bash
python3 ps5_raspike_control.py \
  --socket /tmp/raspike.sock \
  --left-port B \
  --right-port A \
  --arm-port C \
  --force-port D \
  --color-port E \
  --ultrasonic-port F \
  --config ./ps5_controller.yaml \
  --telemetry-host <PC_IP_ADDRESS> \
  --telemetry-port 8765 \
  --web-control-host 0.0.0.0 \
  --web-control-port 8766
```

If you use `start.sh` on the RasPi, pass the PC address through an environment
variable. `start.sh` exposes Web control on the RasPi LAN interface by default:

```bash
RASPIKE_TELEMETRY_HOST=<PC_IP_ADDRESS> ./start.sh
```

`start.sh` also accepts `RASPIKE_LEFT_PORT`, `RASPIKE_RIGHT_PORT`,
`RASPIKE_ARM_PORT`, `RASPIKE_FORCE_PORT`, `RASPIKE_COLOR_PORT`, and
`RASPIKE_ULTRASONIC_PORT`. Set `RASPIKE_ARM_PORT=none` to run without the
optional arm motor. Set `RASPIKE_COLOR_PORT=none` or
`RASPIKE_ULTRASONIC_PORT=none` to skip those sensors. Use
`RASPIKE_INIT_DELAY_SEC` and
`RASPIKE_INIT_RETRIES` to tune startup device setup pacing. `start.sh` defaults
to `RASPIKE_INIT_ORDER=arm-first`; set `RASPIKE_INIT_ORDER=drive-first` to use
the original order.

On shared networks, keep `RASPIKE_WEB_CONTROL_HOST=127.0.0.1` and use SSH port
forwarding instead.

If auto-detection fails:

```bash
python3 ps5_raspike_control.py --event-device /dev/input/eventX
```

Controller can now be connected after process start. The app waits and
auto-reconnects when DualSense appears.

When standard input is an interactive terminal, keyboard control is enabled
automatically. Gamepad and keyboard can be used together; if the controller is
not connected, keyboard input continues to work as the fallback. Use
`--keyboard` to force keyboard input, or `--no-keyboard-fallback` to disable
automatic keyboard input.

```text
W/A/S/D : throttle/steer
Arrow Up/Down: arm motor C up/down
Arrow Left/Right: hub left/right button
x       : hub center button
space   : emergency stop / brake
r       : gyro reset
Enter   : start
c       : motor stop / coast
f       : virtual force sensor touch
```

## Controls

```text
L stick : steering
R2      : accelerator
L2      : brake / reverse
X short : center button
X hold  : emergency stop / brake
Triangle: gyro heading reset
Options : manual start
L1+R1   : toggle safe mode
D-pad Up/Down: arm motor C up/down
D-pad Left/Right: hub left/right button (`dpad_left` / `dpad_right` in yaml)
Circle  : force sensor trigger
Hub left button: left-button action
Hub center button: center-button action
Force sensor touch on `--force-port`: force-sensor trigger action
```

The default input tuning in `ps5_controller.yaml` is GTA5-like: trigger
throttle/brake, left-stick steering, softer high-speed steering, and smoothed
power changes.

## Notes

The bridge gives `libraspike` priority by default. When PTY traffic has been
seen recently, Unix socket writes are dropped for a short window, controlled by
`--pty-priority-ms`.
