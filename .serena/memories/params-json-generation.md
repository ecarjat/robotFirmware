# Parameter JSON Generation

## Structure
- **Source**: `esp32-arduino/tools/param_map.def` - defines all parameters
- **Generated**: `esp32-arduino/cli/params.json` - used by rp_cli.py

## Adding a New Parameter
1. Add entry to `esp32-arduino/tools/param_map.def` following the format:
   ```
   PARAM_ENTRY("name", field_name, "type"),
   ```
   Types: "i8", "u8", "i16", "u16", "i32", "u32", "float"

2. Regenerate `params.json` by running the generation tool from `esp32-arduino/tools/`

## Usage in CLI
The rp_cli.py automatically loads params.json and allows:
- `GET adc_voltage_multiplier` - read current value
- `SET adc_voltage_multiplier 2.5` - set new value
- `SET adc_voltage_multiplier 2.5 SAVE` - set and persist to flash
