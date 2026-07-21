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

The firmware is developed under **Eclipse IDE** and **GNU MCU** plugin. The `script` folder contains Eclipse run/debug configuration files and **JLink** scripts to flash the MCU.

## Target

The SPSWS boards are based on the **STM32L041K6U6** (HW1.0) and the **STM32L081C8T6** (HW2.0) microcontrollers of the STMicroelectronics L0 family. Each hardware revision has a corresponding **build configuration** in the Eclipse project, which sets up the code for the selected board version.

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

## Flash

### Preparation

* **Build** the desired version (with IDE or `cmake`) or **download** a specific [firmware release](https://github.com/Ludovic-Lesur/meteofox-spsws/releases) (expand the `Assets` menu, download the corresponding artifact and extract the binary files from the `zip`).
* **Open the main enclosure** of the weather station.
* Connect the flashing tool to the **P6** (HW1.0) or **P3** (HW2.0) **connector** located in the center of the PCB (standard SWD pinout).

### ST-Link on Nucleo board

* Make sure that the ST-LINK/NUCLEO jumpers (generally designated by **CN2**) are not fitted, in order to **select the external programming connector** instead of the internal MCU.
* An **MSC disk** named `NODE_XXXXXX` should be mounted by the system after USB plugging. If not, download the [ST Cube Programmer](https://www.st.com/en/development-tools/stm32cubeprog.html) software which will install the required drivers. If the MSC disk is still not mounted, follow the ST-Link probe procedure thereafter.
* **Copy/paste** or **click/drop** the `bin` file into the disk.

### ST-Link probe

* Download the [ST Cube Programmer](https://www.st.com/en/development-tools/stm32cubeprog.html) software.
* Launch the software (it might be necessary to run it as **root** or to install specific **USB rules** for the probe to be recognized).
* In the right panel, select `ST-LINK` and click `Connect`.
* Click on the `Open file` tab and select the `hex` file to flash.
* Click on the `Download` button.
* Perform a **memory check** with the `Verify` button located under the `Download` button menu.
* If the operation completed successfully, click on `Disconnect` in the right panel.

### Segger J-Link probe

* Download the [Segger J-Link](https://www.segger.com/downloads/jlink/) software.
* Launch the `JFlashLite` tool.
* Set target device to **STM32L041K6** (HW1.0) or **STM32L081CB** (HW2.0), target interface to **SWD**, speed to **4000kHz** and click `OK`.
* Open the `hex` file to flash.
* Click on the `Program Device` button.

### Final steps

* Check on the platform if the weather station has properly rebooted with the **expected firmware version**.
* **Close the enclosure**.
