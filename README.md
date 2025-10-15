# TML Speaker

A Bluetooth-enabled speaker system with audio jack input, built on ESP32 with LVGL graphical interface.

> **TML** stands for **Tiny Memories Laser**, a registered business (API-led Pty Ltd, ABN) that operated from 2018-2024, providing CO2 laser engraving and cutting services with 200+ products sold online. This speaker project continues the TML brand legacy in the electronics/hardware space, separate from software development work.

## Project Information

- **Author**: Adilson Dias (API-Led Pty Ltd → Tiny Memories Laser (TML))
- **Version**: 1.0.8
- **Date**: 2023-06-05
- **License**: Public Domain (or CC0 licensed)
- **Website**: https://www.tinymemorieslaser.com.au (archived)

## Overview

TML Speaker is an ESP32-based audio system that integrates Bluetooth A2DP audio streaming with traditional audio jack input. The system features an OLED display showing the current audio source and track information, with automatic switching between Bluetooth and aux input. Custom PCB with hand-soldered mixed SMD and through-hole components.

---

⚠️ **DISCLAIMER**: Use this project at your own risk. No guarantees are provided, and no support is available. This is a personal hobby project shared as-is for educational and reference purposes only. Users are responsible for their own implementation, testing, and any consequences arising from use of this project.

---


## Features

### Dual Audio Input
- **Bluetooth A2DP** (Advanced Audio Distribution Profile) sink
- **3.5mm audio jack** input
- **Automatic source switching** via CD4066BE analog switch
- Real-time source detection and switching

### Audio Output
- **3-channel amplification** (Left, Right, Subwoofer)
- **3 x 3W speakers** configuration
- **I2S digital audio** output (PCM5102APWR DAC)
- 44.1kHz sample rate, 16-bit audio

### User Interface
- **128x64 OLED display** (SSD1306)
- Real-time display of:
  - Active audio source (Bluetooth/Jack)
  - Track metadata (title, artist, album, genre)
  - Connection status
- **Potentiometer controls** for volume, bass, and treble adjustment

### Bluetooth Features
- Device name: `TML_SPEAKER_v8`
- AVRCP (Audio/Video Remote Control Profile) support
- Metadata display for playing tracks
- Volume control via remote device
- Secure Simple Pairing (SSP) with PIN: `1234`

## Hardware Specifications

### Microcontroller
| Component | Specification |
|-----------|---------------|
| **Board** | ESP32 (Classic Bluetooth supported) |
| **Flash** | Minimum 4MB recommended |
| **RAM** | 520KB SRAM |

### Display
| Component | Specification |
|-----------|---------------|
| **Type** | OLED |
| **Resolution** | 128x64 pixels |
| **Interface** | SPI |
| **Driver** | SSD1306 |

### Audio Components
| Component | Specification |
|-----------|---------------|
| **DAC** | PCM5102APWR (I2S interface, 24-bit, 192 kHz capable) |
| **Amplifiers** | 3x LM386N-3 (left, right, subwoofer channels) |
| **Analog Switch** | CD4066BE (source switching) |
| **Speakers** | 3x 3W speakers |

### Additional Components
- Potentiometers for volume, bass, and treble adjustment
- Audio jack connector (3.5mm)
- Custom PCB designed in EasyEDA
- Hand-soldered mix of SMD and through-hole components

## Pin Configuration

### I2S Audio Output
```
I2S_DATA_PIN  = GPIO 22  (Serial Data)
I2S_LRCK_PIN  = GPIO 25  (Left/Right Clock)
I2S_BCK_PIN   = GPIO 26  (Bit Clock)
```

### Control Pins
```
BT_CONFIRM_PIN = GPIO 12  (Bluetooth connection confirmation)
DIRECT_OUT_PIN = GPIO 14  (Audio jack output control)
BT_OUT_PIN     = GPIO 33  (Bluetooth output control)
```

### Display (SPI)
```
MOSI = GPIO 23
MISO = GPIO 19
CLK  = GPIO 18
CS   = GPIO 5
DC   = GPIO 4
RST  = GPIO 2
```

## Software Requirements

### Development Framework
- **ESP-IDF**: v4.x or later
- **Language**: C/C++
- **IDE**: VS Code with ESP-IDF extension
- **Compiler**: xtensa-esp32-elf-gcc

### ESP-IDF Components
- FreeRTOS
- ESP32 Bluetooth Classic stack
- ESP32 I2S driver
- NVS (Non-Volatile Storage)

### External Libraries
- **LVGL** (Light and Versatile Graphics Library) - v8.x
- **lvgl_esp32_drivers** - Display and input drivers for ESP32

## Building and Flashing

### Prerequisites
1. Install VS Code
2. Install ESP-IDF extension for VS Code
3. Install ESP-IDF v4.x or later
4. Install CMake 3.5 or later

### Build Steps

1. Clone the repository:
   ```bash
   git clone https://github.com/adilsondias-engineer/TMLSpeaker.git
   cd TMLSpeaker
   ```

2. Configure the project:
   ```bash
   idf.py menuconfig
   ```
   - Select your display configuration (SSD1306, ILI9341, or ST7735)
   - Configure Bluetooth settings
   - Set I2S pins if different from defaults

3. Build the project:
   ```bash
   idf.py build
   ```

4. Identify your ESP32 port:
   ```bash
   # Linux/macOS
   ls /dev/tty.usb*
   
   # Windows
   # Check Device Manager for COM port
   ```

5. Flash to ESP32:
   ```bash
   idf.py -p /dev/ttyUSB0 flash
   ```
   (Replace `/dev/ttyUSB0` with your serial port)

6. Monitor serial output:
   ```bash
   idf.py -p /dev/ttyUSB0 monitor
   ```

### Configuration Options

Multiple display configurations are available in the project:
- `sdkconfig.ssd1306` - For SSD1306 OLED displays (default)
- `sdkconfig.ili9341` - For ILI9341 TFT displays
- `sdkconfig.st7735` - For ST7735 displays

## Project Structure

```
TMLSpeaker/
├── main/
│   ├── main.c              # Main application code
│   ├── bt_app_core.c/h     # Bluetooth application core
│   ├── bt_app_av.c/h       # Bluetooth A2DP/AVRCP handler
│   ├── img_audio_jack1.c   # Audio jack icon
│   ├── img_bt_logo.c       # Bluetooth logo
│   └── CMakeLists.txt
├── components/
│   ├── lvgl/               # LVGL graphics library
│   └── lvgl_esp32_drivers/ # ESP32 display drivers
├── assets/                 # Image assets
├── CMakeLists.txt          # Project CMake configuration
├── sdkconfig               # ESP-IDF configuration
└── README.md              # This file
```

## System Architecture

```
┌──────────────────────┐
│      ESP32          │
│                     │
│   Bluetooth Stack   ├───┐
│                     │   │
└──────────────────────┘   │
                           │
┌──────────────────────┐   │    ┌─────────────────┐    ┌─────────────────────┐
│   Audio Jack Input  ├───┼────►│   CD4066BE      ├───►│   PCM5102A DAC      │
│                     │   │    │    Switch       │    │   (I2S Input)       │
└──────────────────────┘   │    └─────────────────┘    └────────┬────────────┘
                           │                                    │
                           │                                    │ I2S Output
                           │                                    │
┌──────────────────────┐   │    ┌───────────────────────────────┴──────────────┐
│    SSD1306 OLED     │   │    │        LM386N-3 Amplifiers                   │
│    Display (SPI)    │◄──┘    │    (Left, Right, Subwoofer)                  │
│   128x64 pixels     │         └────────────┬─────────────────────────────────┘
└──────────────────────┘                    │
                              ┌─────────────▼─────────────┐
                              │      Speakers (3W x3)     │
                              │   (L, R, Subwoofer)       │
                              └───────────────────────────┘
```

## Usage

### Initial Setup
1. **Power On**: The system initializes with audio jack as the default source
2. **OLED Display**: Shows system status and active audio source

### Bluetooth Connection
1. Search for `TML_SPEAKER_v8` on your Bluetooth device
2. Enter PIN `1234` if prompted
3. Audio automatically switches to Bluetooth when connected
4. Disconnect to revert to jack input

### Playback
- Track information (title, artist, album) displays on OLED screen
- Volume can be controlled from the source device or via Bluetooth remote
- Potentiometers provide analog volume, bass, and treble adjustment

### Audio Source Switching
- **Automatic**: Switches between Bluetooth and audio jack based on connection status
- **Manual**: Via CD4066BE analog switch control (if implemented)

## Hardware Assembly Notes

### PCB Design & Assembly
- **Design Tool**: EasyEDA
- **Assembly**: Hand-soldered mix of SMD and through-hole components
- **PCB Type**: Custom single or double-sided design
- **Component Quality**: Professional-grade audio components for quality fidelity

### Soldering Tips
- **SMD Components**: Use proper flux, fine-tip soldering iron, and solder paste for consistency
- **Through-Hole**: Ensure secure solder joints; use a helping hand for stability
- **Audio Circuits**: Use audio-grade capacitors in amplifier circuits
- **Power Supply**: Use good quality USB-C power adapter or battery with voltage regulation
- **Shielding**: Keep audio signal lines away from high-speed digital lines to minimize noise

## Known Limitations

- Only supports 16-bit audio at 44.1kHz (Bluetooth A2DP limitation)
- Single Bluetooth connection at a time
- No volume control via physical buttons (controlled via Bluetooth source only)
- Limited to LVGL v8.x display rendering capabilities

## Future Enhancements

- Physical volume control knobs integration
- WiFi streaming support
- Battery operation with power management
- Extended metadata display with album art
- Multiple audio sources (USB, SD card)
- Support for higher audio quality profiles

## Troubleshooting

### Bluetooth Connection Issues
- Ensure the device is not paired with multiple devices
- Check that Classic Bluetooth is enabled (not BLE only)
- Verify PIN code `1234` is entered correctly
- Restart both the speaker and source device

### No Audio Output
- Check I2S pin connections (GPIO 22, 25, 26)
- Verify DAC power supply (3.3V)
- Ensure audio source is properly selected (Bluetooth vs Jack)
- Test with oscilloscope to confirm I2S signal presence
- Check speaker impedance and power rating

### Display Not Working
- Verify SPI connections to SSD1306
- Check I2C address configuration in menuconfig (typically 0x3C or 0x3D)
- Ensure LVGL drivers are properly initialized
- Try alternative display configurations (ILI9341, ST7735)

### Audio Distortion
- Reduce I2S volume level in software
- Increase amplifier input coupling capacitor values
- Ensure stable power supply with adequate filtering
- Check speaker connections for loose joints

### Source Not Switching Automatically
- Verify CD4066BE switch connections
- Check control pin GPIO 14 and GPIO 33 voltage levels
- Test switching logic in `bt_app_av.c`

## Performance Considerations

- **Latency**: I2S → DAC → Analog provides minimal latency for real-time audio
- **Audio Quality**: 16-bit/44.1kHz from Bluetooth A2DP standard; DAC capable of 24-bit/192kHz
- **Power Consumption**: Optimized for continuous operation; battery mode in future versions
- **Bluetooth Range**: Typical 10-20 meters depending on antenna design

## Development Tips

- Monitor serial output via VS Code ESP-IDF extension for debugging
- Use ESP32's internal ADC for potentiometer reading with averaging filters
- I2S timing is critical for audio quality—verify with oscilloscope if issues arise
- PCM5102A DAC is I2S-native; minimal software configuration needed
- LVGL display updates can impact audio performance; optimize refresh rates
- Bluetooth AVRCP callbacks provide track metadata in `bt_app_av.c`

## Credits

- **PCB Design**: Adilson Dias using EasyEDA
- **Component Assembly**: Hand-soldered by Adilson Dias
- **LVGL**: https://lvgl.io/
- **ESP-IDF**: Espressif Systems
- **Bluetooth Integration**: ESP32 Classic Bluetooth Stack (Espressif)

## Contact

**Tiny Memories Laser (TML)**  
API-Led Pty Ltd  
[GitHub](https://github.com/adilsondias-engineer)

---

*This project demonstrates integration of ESP32 Bluetooth Classic stack, I2S audio output, LVGL graphics, and multi-channel audio amplification for creating a modern, feature-rich speaker system.*