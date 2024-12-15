# DDS UART-I2C Controller

This Verilog project implements a DDS controller module with UART and I2C interfaces, allowing configuration of waveform type, frequency, and phase via UART commands. The I2C interface supports saving and retrieving configurations from EEPROM.

## Features

- UART interface for receiving commands and data.
- I2C interface for accessing EEPROM to save and load DDS configuration.
- Supports multiple DDS waveform types, frequency, and phase adjustments.
- Includes default configurations for ease of use.
- 5 ms delay mechanism for EEPROM write stabilization.

## Module Details

### Inputs
- `Clk`: System clock (50 MHz).
- `Reset_n`: Active-low reset signal.
- `uart_rx`: UART receive signal.

### Outputs
- `uart_tx`: UART transmit signal.
- `i2c_sclk`: I2C clock signal.
- `i2c_sdat`: I2C data signal (inout).
- `DDS_Data`: 14-bit DDS output data.

## Commands

The module processes UART commands to configure DDS parameters or interact with EEPROM.

### Command List

| Command | Description                                           |
|---------|-------------------------------------------------------|
| `0x30`  | Select base address group 1                          |
| `0x31`  | Select base address group 2                          |
| `0x32`  | Select base address group 3                          |
| `0x33`  | Select base address group 4                          |
| `0x10`  | Read DDS configuration from EEPROM                   |
| `0x20`  | Configure DDS with new waveform, frequency, and phase |

### Configuration Details

When sending `0x20` to configure the DDS, the following data bytes must be sent sequentially:

1. Waveform type (1 byte): Specifies the DDS waveform type.
2. Frequency word high byte (1 byte).
3. Frequency word low byte (1 byte).
4. Frequency word lower high byte (1 byte).
5. Frequency word lower low byte (1 byte).
6. Phase word high byte (1 byte).
7. Phase word low byte (high nibble for phase, low nibble for waveform).

### EEPROM

The I2C EEPROM stores DDS configuration in memory groups. Base addresses for the groups are:

- Group 1: `0x000A`
- Group 2: `0x0010`
- Group 3: `0x0016`
- Group 4: `0x001C`

Each configuration includes waveform type, frequency word, and phase word.

## Default Values

- Frequency Word: `0x00A00000`
- Phase Word: `0x000`
- Waveform Type: Sine wave (`0x00`)

## File Structure

- `dds_uart_i2c_controller.v`: Main Verilog module implementing the controller.
- `DDS_Module.v`: DDS core module.
- `uart_byte_rx.v`: UART receiver module.
- `i2c_control.v`: I2C interface module.

## Usage

1. Clone the repository:

   ```bash
   git clone https://github.com/your_username/dds_uart_i2c_controller.git
   ```

2. Synthesize the module using your preferred FPGA development environment.

3. Connect the UART and I2C peripherals to the respective signals.

4. Send UART commands to configure or retrieve DDS settings.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contributions

Contributions are welcome! Feel free to open an issue or submit a pull request if you have suggestions or improvements.

