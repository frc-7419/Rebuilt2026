# Shooting while moving

Moving-shot aiming is now the normal auto-aim behavior for stationary shots, driving, and turning.
No dashboard mode selection is required after startup. `AutoAim/ActiveMode` reports `MOVING`.
The calculator automatically uses its stationary fast path when stopped. The shoot trigger still
controls shooting. Legacy aiming, static-only mode, and comparison calculations have been removed.

Feeder commands, their existing RPM/angle tolerances, stationary funnel mathematics, gains,
geometry, shooter handoff settings, and physical turret limits are preserved. The stricter
`ShotCalculator/Readiness/*` outputs are diagnostics only; they do not stop feeding. In particular,
a held shot with `Valid=false` does not introduce a new feeder interlock.

## Dashboard controls

| Control | Default | Behavior |
| --- | --- | --- |
| `AutoAim/ReleaseDelaySec` | 0 | Additional solve-to-release delay, separate from sensor age; valid range 0–0.5 s. Measure before increasing. |
| `AutoAim/LaunchVelConstant` | Existing value | Existing launch-speed-to-wheel-RPM conversion; retain the proven stationary value initially. |
| `AutoAim/TurretOmegaAlpha` | 1 | Existing tuning scale on chassis counter-rotation. Target-bearing tracking is added independently. |
| `AutoAim/UseMeasuredCalibration` | false | Latches while disabled. Requires a complete measured table; logs whether each solution actually used it. |
| `AutoAim/DetailedTrajectory` | false | Enables expensive trajectory arrays on the real robot. SIM/REPLAY still show trajectories. |

Tracking history resets on enable, interrupted auto-aim, pose resets, target changes, winding
changes, calibration changes, timing gaps, and invalid inputs.
The first derivative sample is zero; chassis counter-rotation is available immediately.

## What to inspect before moving shots

1. Take a log with stationary shots, driving, turning, and all cameras running. Do not interpret
   desktop test timing as roboRIO timing.
2. Inspect `Robot/PeriodicTimeMs`, AdvantageKit's loop timing, `AutoAim/UpdateTimeMs`, and
   `ShotCalculator/SolveTimeMs`. The full 20 ms loop must have headroom.
3. Inspect `Odometry/SampleAgeSec`, `AcceptedSamples`, `RejectedSamples`, `QueueOverflows`,
   `SamplerErrors`, and `SamplerStatus`. Missing frames must not create fresh pose timestamps.
   The hardware sampler keeps its original 250 Hz CAN FD / 100 Hz CAN rate. A disconnected gyro
   permits wheel-derived rotation; required module signals must still be usable.
4. Inspect `Robot/RioCANUtilization`, `Drive/CANivoreUtilization`, `Drive/CANivoreStatus`,
   `Turret/FusedSensorUnlicensed`, and `Turret/RemoteSensorInvalid`. Confirm no new bus errors or
   fused-sensor faults. Hood position/velocity now report at 50 Hz and turret feedback at 100 Hz;
   slow electrical diagnostics and existing encoder configuration are retained. Verify licensing
   and remote encoder signal configuration on the robot; the code does not change the feedback ratios.
5. Inspect the absolute `ShotCalculator/*` setpoints and actual mechanism positions. A `Reason`
   other than `OK` explains why the calculator holds targets.

`ShotCalculator/Valid` requires finite/fresh pose data, a converged solution, the funnel constraint
for model-based hub shots, and a reachable turret angle. Pose age above 100 ms is rejected.
Invalid input holds the last finite targets with zero turret feedforward. Before any valid solution,
the fallback is current bounded turret angle, max hood, and zero RPM. The obstacle hood override
still takes priority. Existing RPM caps remain in effect; actual accuracy must be measured.

## Practice sequence

1. Record a stationary baseline at representative close, medium, and far working distances. Use
   at least 30 shots per condition and record hits, distance, RPM, hood, and battery voltage.
2. Keep the current working tuning as the baseline. After any tuning or hardware changes, repeat
   stationary shots before increasing speed. Use the previous working Git revision for rollback.
3. Strafe left/right at 1 m/s, drive toward/away, rotate both directions, and combine translation
   with gentle turns. Increase translation to 2 m/s only after the lower-speed cases behave correctly.
4. Use at least 30 shots per condition. Initial acceptance: moving accuracy within 10 percentage
   points of the matching stationary baseline, without repeated loop overruns or stale poses.
   Check individual motion directions rather than averaging away a directional error.
5. Record target/actual turret and hood angles, turret velocity feedforward, wheel RPM error,
   pivot velocity, virtual target, flight time, iteration count, and readiness diagnostics.
   Pure turning must counter-rotate; strafing must lead opposite the inherited launch velocity.
6. Measure solve-to-ball-release delay and ball flight time separately with timestamped logs and
   slow-motion video. Tune release delay first if tracking is consistently behind. Do not change
   stationary launch calibration to conceal a motion-only error.

Robot accuracy, release delay, bus load, licensed fused-feedback operation, and roboRIO timing
cannot be verified by desktop tests and remain practice checks after changes to tuning or hardware.

## Optional measured calibration

Populate `ShotCalibration.MEASURED` with at least two `Point(distanceM, hoodRad, wheelRpm,
flightTimeSec)` entries, ordered by increasing distance. `distanceM` uses this code's existing
horizontal pivot-to-target distance minus the 0.28 m launch offset, not bumper-to-hub distance.
Measure flight time from ball exit to the target height used by the shot model. Use radians for
hood angle, wheel RPM (not rotor RPM), and seconds for flight time.

All four quantities must be present and finite. Angle/RPM must respect configured limits and
flight time must be positive. The table is opt-in and interpolates only inside its measured range;
outside it, the original model is used. Passing always uses its existing model. Never copy another
robot's RPM/hood settings, invent flight times, or treat `ShotWarmstartTable` as measured data.

## Automated verification

Run `./gradlew test -x spotlessApply -x createVersionFile` for the regression suite without source
formatting/version generation. Tests cover stationary parity, motion convergence and direction,
release-pose prediction, wrap handling, travel limits, invalid/stale holds, reset behavior,
calibration boundaries, immediate motor requests and shooter handoff, and odometry timing/fusion.

## Research references

- [Mechanical Advantage's 2026 LaunchCalculator](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/5faa7595c81d8cf7bd28dec9aa9431c96ad9d86b/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java): shared cached parameters, prediction, calibrated flight time, and bounded iteration. Their drivetrain aiming and calibration values are robot-specific.
- [Team 6940's ProjectileCalculator](https://github.com/Team6940/2026Rebuilt/blob/a20b94162a2a0e3555f5c7e63021b9e24981619e/src/main/java/frc/robot/util/ProjectileCalculator.java) and [HybridShootCommand](https://github.com/Team6940/2026Rebuilt/blob/a20b94162a2a0e3555f5c7e63021b9e24981619e/src/main/java/frc/robot/commands/HybridShootCommand.java): turret virtual-target iteration and tracking feedforward.
- [Team 360's findings](https://github.com/FRCTeam360/RainMaker26/blob/main/docs/advanced_topics/ShootOnTheMove.md): their documented zero flight-time placeholder illustrates why a compensation architecture alone does not establish working moving shots.
- [CTRE status signals](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html): feedback frequency, bus utilization, and remote-sensor signal requirements.

The new implementation uses these architectural ideas with this robot's existing solver; it does
not import their gains, lookup values, or hardware assumptions.
