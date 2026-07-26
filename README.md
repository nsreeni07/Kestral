# Kestral

Kestral is a custom flight computer and thrust-vector-control (TVC) system for an active-guidance model rocket. It's a from-scratch embedded C++ stack — sensor fusion, a finite-state flight sequencer, a dual-axis PID/servo control loop, and buffered SD logging — designed to run on an STM32F7 and validated entirely in software before it ever touches real hardware, via a native PC simulation harness and an interactive flight-data dashboard.

This repo contains two things:

Folder	Purpose

Flight Code/	The actual flight firmware — the code that runs on the vehicle

Simulation Files/	A PC-native simulation harness that runs the real flight logic against a synthetic rocket flight, plus an HTML dashboard for visualizing the results


Why this exists

Most hobby TVC projects either hardcode control logic directly into loop(), or can only be tested by literally launching the rocket. Kestral is built around two rules instead:

main.cpp is intentionally thin. It initializes hardware, wires modules together, registers tasks with a cooperative scheduler, and does nothing else. Every real behavior — sensor fusion, state transitions, control math — lives in its own module, independent of any specific microcontroller.
The flight logic must run identically on a laptop and on the flight computer. Because the peripheral drivers are cleanly separated from the control/estimation code, the exact same MadgwickFilter, AltitudeEstimator, FlightStateMachine, and TVCController classes that fly on the rocket can be linked into a plain PC binary and fed a simulated flight. That's what Simulation Files/sim_flight.cpp does — it's not a re-implementation or a mock, it's the production code under test.
System architecture
                     ┌─────────────────────────────────────────┐
                     │              main.cpp                    │
                     │   (init hardware → wire modules →         │
                     │    register scheduler tasks → loop)       │
                     └───────────────────┬───────────────────────┘
                                         │
        ┌────────────────┬───────────────┼───────────────┬────────────────┐
        │                │               │               │                │
   1000 Hz           50 Hz           500 Hz          100 Hz            10 Hz
  taskImuRead     taskBaroRead   taskControlLoop    taskLogData     taskStatusUpdate
        │                │               │               │                │
        ▼                ▼               ▼               ▼                ▼
  ┌───────────┐   ┌──────────────┐  ┌────────────┐  ┌────────────┐  ┌───────────────┐
  │ ICM-45686 │   │   BMP580     │  │ FlightState │  │FlightLogger│  │  StatusLED /  │
  │  (IMU)    │   │ (barometer)  │  │  Machine +  │  │  (SD, CSV) │  │ FaultManager /│
  │    ↓      │   │      ↓       │  │TVCController│  │            │  │   Watchdog    │
  │Madgwick   │   │  Altitude    │  │             │  │            │  │               │
  │ Filter    │   │  Estimator   │  │             │  │            │  │               │
  └───────────┘   └──────────────┘  └────────────┘  └────────────┘  └───────────────┘

A single cooperative Scheduler (driven by a 1 ms hardware timer tick) runs five tasks at five different rates, all sharing one FlightData snapshot struct rather than passing data through queues — appropriate for a single-core control loop where every task runs to completion before the next one starts.

#Module breakdown
#Module	           #Responsibility
sensors.*         	Raw hardware drivers for the ICM-45686 6-axis IMU (SPI) and BMP580 barometer (I2C). Return scaled physical units only — no fusion math.
ahrs.*	            Sensor fusion. MadgwickFilter turns gyro + accel into a roll/pitch/yaw estimate; AltitudeEstimator turns barometric pressure into AGL altitude and a filtered vertical velocity.
state_machine.*	    The flight sequencer: IDLE → ARMED → LAUNCH_DETECTED → POWERED_ASCENT → COAST → APOGEE → DESCENT → LANDED, with a FAULT state reachable from anywhere. Transitions require a sustained condition (not a single noisy sample) before triggering.
tvc_control.*	      Two independent PID loops (pitch, yaw) targeting 0° attitude error, each feeding a rate-limited ServoMixer before commanding the servo. Only ever allowed to actuate during POWERED_ASCENT.
peripherals.*	      "Dumb" hardware I/O: servo PWM output, a WS2812 status LED, SD card block access, battery voltage ADC, BQ25883 charger status. No control logic lives here.
flight_logger.*	    Buffers flight data as CSV rows in RAM and flushes to the SD card in blocks (to avoid blocking the control loop on slow writes), plus a FaultManager that logs faults, drives LED blink codes, and can force the vehicle into FAULT.
scheduler.*	        Minimal cooperative task scheduler driven by a hardware timer tick.
config.h	          Single source of truth for every tunable constant and pin assignment — PID gains, loop rates, detection thresholds, servo limits, Madgwick beta schedule, pin map.



Flight state machine & safety interlocks

The state machine is fed already-computed values (acceleration magnitude, altitude, vertical velocity) each control tick and owns only transition timing logic, which keeps it fully unit-testable with synthetic inputs. Key safety property: tvcAuthorized() returns true only during POWERED_ASCENT — this is the single authoritative gate the TVC controller checks every tick. Outside that window (on the pad, during coast, under canopy, after landing) the servos are held centered and the PID/rate-limiter state is reset so there's no integral windup or derivative kick if TVC becomes authorized again later in the same flight.

Sensor fusion gain scheduling

The Madgwick filter's beta gain (the accelerometer-vs-gyro trust trade-off) is scheduled by flight state rather than fixed:

High beta on the pad — fast convergence while the accelerometer is still a trustworthy gravity reference.
Low beta during powered ascent — motor thrust contaminates the accelerometer, so the filter leans on the gyroscope instead.
Low-moderate beta during coast — still mostly gyro-driven.
Higher beta during descent — back to trusting the accelerometer once thrust ends.
Control loop

Each axis (pitch, yaw) runs an independent PID controller with a 0° setpoint (i.e., "point straight up relative to the current thrust axis"), anti-windup via integral clamping, and derivative-on-error (safe here since the setpoint never changes). The PID output is passed through a ServoMixer that clamps to the mechanical deflection limits and rate-limits the commanded change per control tick to prevent violent servo slew, before being converted to a PWM pulse width and sent to the servo driver.

Hardware target
Component	Part	Interface
MCU	STM32F7 (HAL-based)	—
IMU	ICM-45686 (6-axis accel + gyro)	SPI
Barometer	BMP580	I2C
TVC actuation	2x RC servo (pitch, yaw)	Timer PWM
Status indicator	WS2812 addressable LED	Timer PWM + DMA
Storage	microSD (FatFs)	SDIO
Power	2S Li-ion + BQ25883 charger	ADC + I2C

main.cpp declares the CubeMX-generated peripheral handles (hspi1, hi2c1, htim1, etc.) as extern placeholders — in a real CubeIDE/PlatformIO project these come from the CubeMX-generated init calls, which must run before the driver objects are constructed. Pin assignments and register-level details all live in config.h and sensors.cpp/peripherals.cpp, ready to be re-pointed once the physical board is finalized.

Running the simulation

The simulation harness (Simulation Files/sim_flight.cpp) links the exact same estimation/control classes used on the flight computer against a synthetic sensor feed — no STM32 toolchain required, since the HAL calls in peripherals.cpp are stubbed no-ops on a PC build.

The synthetic flight it generates:

Time	Phase
0.0 – 0.3 s	On the pad (ARMED), resting at 1 g with small noise
0.3 – 2.0 s	POWERED_ASCENT, ~6 g net acceleration, with an injected pitch/yaw disturbance (simulating tip-off / wind) that the TVC loop has to null out
2.0 – 8.0 s	COAST, decelerating under gravity and drag
~8.0 s	APOGEE
8.0 – 14 s	DESCENT under drogue/main at a simple constant descent rate
14 s+	LANDED

Build and run it with a plain host compiler:

bash
cd "Simulation Files"
g++ -std=c++17 -I"../Flight Code" sim_flight.cpp \
    "../Flight Code/ahrs.cpp" \
    "../Flight Code/state_machine.cpp" \
    "../Flight Code/tvc_control.cpp" \
    "../Flight Code/peripherals.cpp" \
    -o sim_flight
./sim_flight > flight_log.csv

The program streams a CSV to stdout with one row per 500 Hz control tick: simulated time, flight state, true vs. estimated altitude/velocity, acceleration magnitude, roll/pitch/yaw, PID outputs, and commanded servo angles.

Flight Dashboard

Simulation Files/Flight Dashboard.html is a self-contained interactive dashboard for visualizing a sim_flight CSV run (or real logged flight data in the same format). Open it directly in a browser. It plots several time-series panels; three of them are described below.

TVC PID Output & Servo Command

Plots PID Output Pitch, Servo Pitch Cmd, and PID Output Yaw over the full simulated flight. Both signals are non-zero only during POWERED_ASCENT — everywhere else they sit flat at zero, because TVCController::update() forces the servos to center and zeroes its outputs whenever tvcAuthorized() is false. In the captured run, the pitch PID output rises sharply to correct the injected tip-off disturbance (peaking near 12°, the configured OUTPUT_LIMIT_DEG), decays as the error is nulled out, and shows a couple of transient spikes right at burnout as acceleration drops and the controller reacts to the changing measurement — right before TVC authorization is revoked and both traces snap to zero for the rest of the flight.

Attitude (Madgwick AHRS Output)

Shows the real MadgwickFilter class's roll/pitch/yaw estimate across the whole flight, from the fused gyro + accelerometer data. Roll and yaw drift outward during coast (expected — with no aerodynamic correction, unguided drift accumulates once TVC deauthorizes at burnout and the beta gain has dropped to trust the gyro more heavily), pitch swings the opposite direction as the simulated vehicle continues to weathercock/tumble through coast, and all three angles collapse back toward zero as the descent-phase beta gain (which trusts the accelerometer again) pulls the estimate back to a stable reference under canopy.

Altitude & Vertical Velocity

Compares true simulated altitude (ground-truth physics) against the AltitudeEstimator's barometric-derived estimate — the two traces are effectively identical throughout, since the estimator uses only the pressure-derived altitude with no injected barometer noise in this run, so the filter tracks perfectly. The altitude curve climbs through powered ascent and coast, peaks at apogee (~138 m in this run), and shows the characteristic two-slope descent: a faster drop under drogue followed by a shallower rate after main deployment, before leveling off at landing. The vertical velocity trace crosses zero at apogee — the exact signal the state machine watches, requiring a sustained negative reading for APOGEE_CONFIRM_MS before confirming the transition out of COAST, so a single noisy sample near the apex can't trigger a false transition.

Data logged

Every row written to the CSV (both by the real FlightLogger on hardware and by the simulation) includes: elapsed time, flight state, true and estimated altitude, true and estimated vertical velocity, acceleration magnitude, roll/pitch/yaw, PID outputs for pitch and yaw, and the resulting commanded servo angles — enough to fully reconstruct and debug a flight after the fact, or feed into the dashboard above.

Fault handling

A central FaultManager owns the "what happens when something breaks" policy: every fault is logged, reflected as a distinct LED blink pattern (so a fault code is readable without a display or serial link, e.g. for post-flight/pad diagnostics), and critical faults (sensor init failure, comms timeout, SD failure) immediately force the state machine into FAULT — which, like every non-POWERED_ASCENT state, also forces the servos to center and disables TVC.

Status / roadmap

This is a software-first project: the control, estimation, and sequencing logic is implemented and validated in simulation. Pin assignments and CubeMX peripheral initialization are placeholders pending finalization of the physical board. Suggested next steps for anyone building on this:

Wire up the real CubeMX-generated main.h/HAL init calls in place of the extern placeholders in main.cpp.
Bench-test each sensor driver individually against real hardware before a full integration test.
Static-fire test the TVC mount + servo response against sim_flight's injected disturbance profile before any powered flight.
Tune Config::PID gains against real vehicle mass properties and TVC mount geometry — the values in config.h are simulation-tuned starting points, not flight-proven.
Safety

This is flight-critical embedded software controlling an actively-thrusting rocket. Follow all applicable model rocketry safety codes (e.g. NAR/Tripoli high-power rocketry safety codes) and local regulations for any TVC or active-guidance vehicle. Ground-test extensively — static fires, bench servo tests, and full sim_flight regression runs — before any powered flight, and never fly with unresolved critical faults.
