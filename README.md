# Description

The aim of the **MeteoFox** project was to design an autonomous weather station using solar energy and Sigfox connectivity. The **SPSWS** is the main processing board of the device.

# Hardware

The boards were designed on **Circuit Maker V1.3**. Below is the list of hardware revisions:

| Hardware revision | Description  | `cmake_hw_version` | Status |
|:---:|:---:|:---:|:---:|
| [SPSWS HW1.0](https://365.altium.com/files/C5470066-C92D-11EB-A2F6-0A0ABF5AFC1B) | Initial version for prototyping. | `HW1_0` | :x: |
| [SPSWS HW2.0](https://365.altium.com/files/C7B06FC0-C92D-11EB-A2F6-0A0ABF5AFC1B) | PCB fitted to outdoor casing.<br>Add connectors to external sensors and antennas.<br>Separate SPI interfaces for radio and ADC.<br>Improved power tree.<br>Shielding added on the radio circuitry. | `HW2_0` | :white_check_mark: |

The weather station is also composed of 2 daughter boards which embed the meteorological sensors. These boards are located outdoor in a dedicated casing, out of the main enclosure.

 Hardware revision | Description | Status |
|:---:|:---:|:---:|
| [MPMCM HW1.0](https://365.altium.com/files/CA4F6A2D-C92D-11EB-A2F6-0A0ABF5AFC1B) | Initial version with all sensors. | :x: |
| [THPSM HW1.0](https://365.altium.com/files/C8C019CC-C92D-11EB-A2F6-0A0ABF5AFC1B) | Initial version. | :x: |
| [THPSM HW1.1](https://365.altium.com/files/C6225D4D-C92D-11EB-A2F6-0A0ABF5AFC1B) | Change form factor. | :white_check_mark: |
| [LUSM HW1.0](https://365.altium.com/files/C461191C-C92D-11EB-A2F6-0A0ABF5AFC1B) | Initial version. | :x: |
| [LUSM HW1.1](https://365.altium.com/files/C5607DB6-C92D-11EB-A2F6-0A0ABF5AFC1B) | Change form factor. | :white_check_mark: |

# Embedded software

## Environment

As of version `sw1.2.3` the embedded software is developed under **Eclipse IDE** version 2024-09 (4.33.0) and **GNU MCU** plugin. The `script` folder contains Eclipse run/debug configuration files and **JLink** scripts to flash the MCU.

> [!WARNING]
> To compile any version under `sw3.0`, the `git_version.sh` script must be patched when `sscanf` function is called: the `SW` prefix must be replaced by `sw` since Git tags have been renamed in this way.

## Target

The SPSWS boards are based on the **STM32L041K6U6** (HW1.0) and the **STM32L081C8T6** microcontroller of the STMicroelectronics L0 family. Each hardware revision has a corresponding **build configuration** in the Eclipse project, which sets up the code for the selected board version.

## Structure

The project is organized as follow:

* `drivers` :
    * `device` : MCU **startup** code and **linker** script.
    * `registers` : MCU **registers** address definition.
    * `peripherals` : internal MCU **peripherals** drivers.
    * `components` : external **components** drivers.
    * `utils` : **utility** functions.
* `middleware` :
    * `analog` : High level **analog measurements** driver.
    * `cli` : **AT commands** implementation.
    * `gps` : High level **GPS** driver.
    * `power` : Board **power tree** manager.
    * `sigfox` : **Sigfox EP_LIB** and **ADDON_RFP** submodules and low level implementation.
* `application` : Main **application**.

## Sigfox library

**Sigfox technology** is very well suited for this application for 3 main reasons:

* **Data quantity is low**, weather data can be packaged on a few bytes and does not require high speed transmission.
* **Low power** communication enables **energy harvesting** (solar cell + supercap in this case), so that the device is autonomous.
* The weather station can be placed is very isolated places thanks to the **long range** performance.

The project is based on the [Sigfox end-point open source library](https://github.com/sigfox-tech-radio/sigfox-ep-lib) which is embedded as a **Git submodule**.

## Build

The project can be compiled by command line with `cmake`.

```bash
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE="script/cmake-arm-none-eabi/toolchain.cmake" \
      -DTOOLCHAIN_PATH="<arm_none_eabi_gcc_path>" \
      -DSPSWS_HW_VERSION="<cmake_hw_version>" \
      -DSPSWS_MODE_CLI=OFF \
      -DSPSWS_WIND_RAINFALL_MEASUREMENTS=ON \
      -DSPSWS_WIND_VANE_ULTIMETER=OFF \
      -DSPSWS_SEN15901_EMULATOR=OFF \
      -G "Unix Makefiles" ..
make all
```
