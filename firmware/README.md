# ATmega328PB Marine Compressor Controller Firmware

This firmware implements a simple compressor clutch controller for a custom Volvo Penta replacement board based on the ATmega328PB-AU.

## Implemented behavior

- Measures engine speed from `RPM_SIG` on `PD2 / INT0`.
- Assumes a 66-tooth flywheel with 1 pulse per tooth.
- Computes RPM from tooth period using:
  - `RPM = (timer_frequency * 60) / (tooth_period_ticks * 66)`
- Uses `PC0 / ADC0` to map a 10k linear potentiometer to a disengage threshold from 2200 RPM to 2800 RPM.
- Uses `PC1 / ADC1` as an active-high kickdown input.
- Drives `PB1` to enable the compressor clutch MOSFET output stage.
- Enables the watchdog (`250 ms`) and expects the main loop to continue running.
- Applies hysteresis and a hard safety cutoff to avoid chatter and overspeed clutch operation.

## Default control thresholds

- Normal clutch engage: `1700 RPM`
- Kickdown clutch engage: `1200 RPM`
- Pot-adjusted disengage: `2200..2800 RPM`
- Hard safety cutoff: `3000 RPM`
- Low-side hysteresis release: `120 RPM`

These constants are near the top of `compressor_controller.c` for easy adjustment.

## Notes

- Brown-out protection must be configured in the AVR fuse settings.
- The code assumes the VR front-end already converts the sensor waveform into a clean digital pulse train.
- `DEBUG_UART` is disabled by default. Set it to `1` if you want periodic serial telemetry on `PD1/TX` at `115200 baud`.
