# Kestral

A custom flight computer and thrust-vector-control (TVC) system for a model rocket, built in embedded C++ for an STM32F7. It handles sensor fusion, flight-phase detection, closed-loop TVC, and data logging — and can be fully tested on a laptop before it ever flies.

```
Flight Code/         → the actual flight firmware
Simulation Files/    → PC simulation of the same code + an HTML dashboard
```

## How it works

A cooperative scheduler runs five tasks at different rates, all sharing one data snapshot:

- **1000 Hz** — read IMU, update Madgwick attitude filter
- **500 Hz** — update flight state machine, run TVC control loop
- **100 Hz** — log data to SD
- **50 Hz** — read barometer, update altitude/velocity estimate
- **10 Hz** — update status LED, feed watchdog

**Flight sequence:** `IDLE → ARMED → LAUNCH_DETECTED → POWERED_ASCENT → COAST → APOGEE → DESCENT → LANDED`, with a `FAULT` state reachable from anywhere. Each transition requires a sustained condition (not just one noisy sample) to avoid false triggers.

**TVC control:** two PID loops (pitch, yaw) try to hold 0° attitude error, with output rate-limited before hitting the servos. Critically, **the servos are only ever active during `POWERED_ASCENT`** — everywhere else they're held centered and the PID state is reset, so there's no windup or kick if control re-engages later.

**Attitude estimation:** a Madgwick filter fuses gyro + accelerometer data. It trusts the accelerometer more on the pad and during descent, and trusts the gyro more during powered flight and coast (since thrust/drag distort the accelerometer reading).

## Hardware

| Part                     | Interface |
|--------------------------|-----------|
| STM32F7 MCU              |      -    |
| ICM-45686 IMU            |    SPI    |
| BMP580 barometer         |    I2C    |
| 2x servo (pitch/yaw TVC) |    PWM    |
| WS2812 status LED        |  PWM/DMA  |
| microSD logging          |   SDIO    |
| 2S Li-ion                |  ADC/I2C  |
| BQ25883 charger          |  ADC/I2   |


Pin assignments and CubeMX init are still placeholders — swap in real values once the board is finalized.

## Running the simulation

`sim_flight.cpp` links the real flight code against a synthetic flight (pad → 6g powered ascent with an injected disturbance → coast → apogee → descent → landing), so you can test and tune without hardware:

```bash
cd "Simulation Files"
g++ -std=c++17 -I"../Flight Code" sim_flight.cpp \
    "../Flight Code/ahrs.cpp" "../Flight Code/state_machine.cpp" \
    "../Flight Code/tvc_control.cpp" "../Flight Code/peripherals.cpp" \
    -o sim_flight
./sim_flight > flight_log.csv
```

Open `Flight Dashboard.html` in a browser to visualize the resulting CSV.

## Dashboard charts

**TVC PID Output & Servo Command** — pitch/yaw PID output and servo command, both flat at zero except during `POWERED_ASCENT`. In the sample run, pitch spikes up to correct an injected disturbance, then settles as the error is nulled out, before dropping to zero at burnout.

**Attitude (Madgwick AHRS)** — roll/pitch/yaw estimate over the flight. Angles drift during coast (no active correction once TVC turns off), then converge back toward zero during descent as the filter starts trusting the accelerometer again.

**Altitude & Vertical Velocity** — true vs. estimated altitude (barometric), which track closely. Altitude peaks at apogee (~138 m in this run) and comes down in two slopes (drogue, then main). Vertical velocity crossing zero is what the state machine uses to detect apogee.

## Status

Control and estimation logic is implemented and validated in simulation; hardware wiring (CubeMX init, pin assignments) is still pending. Before any powered flight: bench-test each sensor, static-fire test the TVC mount, and re-tune PID gains for your actual vehicle — the current gains are simulation defaults, not flight-proven.

## Safety

This controls an actively-thrusting rocket. Follow NAR/Tripoli (or local) high-power rocketry safety codes, ground-test extensively, and don't fly with unresolved critical faults.
