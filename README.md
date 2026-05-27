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
  --pty-link /tmp/raspike-tty \
  --unix-socket /tmp/raspike.sock \
  --pty-priority-ms 200 \
  -v
```

Point existing `libraspike` code at `/tmp/raspike-tty`.

Start PS5 control:

```bash
python3 ps5_raspike_control.py \
  --socket /tmp/raspike.sock \
  --left-port B \
  --right-port A \
  --max-power 60 \
  -v
```

If auto-detection fails:

```bash
python3 ps5_raspike_control.py --event-device /dev/input/eventX
```

## Controls

```text
X       : emergency stop / brake
Circle  : motor stop / coast
Triangle: gyro heading reset
Square  : log mark / save
L stick : steering
R2/L2   : accelerator / brake-reverse
R1/L1   : increase/decrease power limit
D-pad   : experiment mode selection
OPTIONS : selected experiment start
SHARE   : cancel / return idle
```

Default driving style is car-like:

```text
R2      : accelerator
L2      : brake / reverse
L stick : steering
```

For L-stick-only arcade driving:

```bash
python3 ps5_raspike_control.py --drive-style arcade
```

## Notes

The bridge gives `libraspike` priority by default. When PTY traffic has been
seen recently, Unix socket writes are dropped for a short window, controlled by
`--pty-priority-ms`.

