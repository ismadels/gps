# AGENTS.md

## Project Type
Academic GPS/NMEA course project. No build/test/lint tooling.

## Python Files
- `practica1.py` - NMEA-GGA parsing, serial/file input, UTM conversion
- `practica2_gps.py` - GUI with tkinter + PIL for map display
- `practica3_gps.py` - Extended GPS functionality

## Dependencies
- `pyserial` - serial port communication
- `Pillow` - image rendering in tkinter GUI

## Running Scripts
```bash
python practica1.py --port        # Read from serial port
python practica1.py --file        # Read from file
python practica2_gps.py          # Launch GUI
```

## Configuration
Default serial settings per file:
- Port: COM9 (practica1), COM3 (practica2)
- Baud: 4800
- UTM Zone: 30

Ellipsoids: WGS84, ED50

## VS Code
Uses system Python (ms-python.python:system), not venv.