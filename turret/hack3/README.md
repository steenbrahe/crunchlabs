# CrunchLabs IR Turret - Hack Pack Project

This project contains custom firmware for the [CrunchLabs IR Turret](https://www.crunchlabs.com/products/ir-turret) from the Hack Pack subscription.

## Hardware

- **Board**: Arduino Nano-compatible (ATmega328P) with USB-C
- **Servos**: 3x servo motors (yaw, pitch, roll/fire)
- **IR Receiver**: Connected to pin D9
- **Remote**: NEC protocol IR remote

### Pin Configuration

| Component | Pin |
|-----------|-----|
| IR Receiver | D9 |
| Yaw Servo | D10 |
| Pitch Servo | D11 |
| Roll/Fire Servo | D12 |

## Development Setup

### Prerequisites

1. **VS Code** with **PlatformIO IDE** extension installed
2. **USB-C cable** to connect the turret

### Opening the Project

1. Open VS Code
2. Go to `File → Open Folder`
3. Select the `hack3` folder
4. Wait for PlatformIO to initialize (downloads dependencies automatically)

## Building and Deploying

### Using VS Code GUI

1. Click the **PlatformIO icon** (alien head 👽) in the left sidebar
2. Under **PROJECT TASKS → nano**:
   - **Build** - Compile the code and check for errors
   - **Upload** - Flash the firmware to the turret
   - **Monitor** - Open Serial Monitor to view logs

### Using Command Palette

- `Cmd+Shift+P` → "PlatformIO: Build"
- `Cmd+Shift+P` → "PlatformIO: Upload"
- `Cmd+Shift+P` → "PlatformIO: Serial Monitor"

### Using Terminal (if pio is in PATH)

```bash
# Build
pio run

# Upload to board
pio run -t upload

# Monitor serial output
pio device monitor
```

## Monitoring Serial Output

The turret outputs debug information via Serial at **9600 baud**. To view:

1. Click the **plug icon** 🔌 in the PlatformIO toolbar (bottom of VS Code)
2. Or: `Cmd+Shift+P` → "PlatformIO: Serial Monitor"

### Serial Output Examples

```
START /src/main.ino from Jan 31 2026
Ready to receive IR signals of protocols: NEC
at pin 9
HOMING
```

When using the remote:
```
LEFT
RIGHT
UP
DOWN
FIRING
```

### Passcode Feature

This firmware includes a passcode lock. Enter `1221` using the remote's number buttons to unlock movement and firing controls. You'll see:
- **YES** (head nods) - Correct passcode
- **NO** (head shakes) - Incorrect passcode

## IR Remote Commands

| Button | Action |
|--------|--------|
| ↑ Up | Pitch up |
| ↓ Down | Pitch down |
| ← Left | Rotate left |
| → Right | Rotate right |
| OK | Fire single dart |
| * Star | Fire all darts |
| 1, 2 | Passcode entry |

## Troubleshooting

### Upload Failed

1. Check the USB cable is connected
2. Verify the port in `platformio.ini` matches your device:
   ```bash
   ls /dev/cu.usb*
   ```
3. Update `upload_port` if needed

### Wrong Board?

If upload fails with "programmer not responding", try changing the board:

```ini
; Try old bootloader variant
board = nanoatmega328
```

### Serial Monitor Shows Garbage

Ensure `monitor_speed = 9600` matches the code's `Serial.begin(9600)`

## Resources

- [CrunchLabs IR Turret Product Page](https://www.crunchlabs.com/products/ir-turret)
- [CrunchLabs Web IDE](https://ide.crunchlabs.com/editor/ir-turret)
- [IRremote Library Documentation](https://github.com/Arduino-IRremote/Arduino-IRremote)
- [PlatformIO Documentation](https://docs.platformio.org/)

## License

MIT License - See source file for full license text.
