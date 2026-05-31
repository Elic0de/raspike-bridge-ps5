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
  --max-power 60 \
  --config ./ps5_controller.yaml \
  -v
```

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
space   : emergency stop / brake
r       : gyro reset
Enter   : start
c       : motor stop / coast
```

## Controls

```text
L stick : steering
R2      : accelerator
L2      : brake / reverse
X short : center button (mapped to start/resume)
X hold  : emergency stop / brake
Triangle: gyro heading reset
Options : manual start
L1+R1   : toggle safe mode
D-pad Up/Down: arm motor C up/down
D-pad Left/Right: left button press
Circle  : force sensor trigger
```

The default input tuning in `ps5_controller.yaml` is GTA5-like: trigger
throttle/brake, left-stick steering, softer high-speed steering, and smoothed
power changes.

## Notes

The bridge gives `libraspike` priority by default. When PTY traffic has been
seen recently, Unix socket writes are dropped for a short window, controlled by
`--pty-priority-ms`.
