## Quick orientation for AI coding assistants

This repository is FRC Team 7419's 2026 Java robot code built on WPILib + AdvantageKit. The goal of these notes is to get an AI assistant immediately productive: where to look, how to build/run, and project-specific patterns to follow.

Key locations
- `build.gradle` — primary build, dependencies (WPILib, AdvantageKit), Spotless formatting, and custom Gradle tasks (e.g., `replayWatch`).
- `README.md` — team conventions (naming, branching) and AdvantageKit notes.
- `src/main/java/frc/robot/` — main code. Important subfolders:
  - `subsystems/` — hardware-logic split into AdvantageKit IO interfaces and implementations.
  - `commands/` — command-based actions that drive subsystems.
  - `simulation/` and `util/` — sim helpers and shared utilities.
- `vendordeps/` — JSON files with vendor library versions (e.g., AdvantageKit.json, Phoenix6.json).
- `src/main/deploy/` — static files packaged to `/home/lvuser/deploy` on the robot (PathPlanner autos, navgrid, config JSONs).

Big-picture architecture (what to read together)
- AdvantageKit IO pattern: each subsystem exposes an `XxxIO` interface and typically has `XxxIOTalonFX` (real) and `XxxIOSim` (simulation) implementations. Example: `src/main/java/frc/robot/subsystems/intake/IntakeIO.java`, `IntakeIOTalonFX.java`, `IntakeIOSim.java`.
- Subsystem logic is hardware-agnostic: commands call subsystem methods; IO implementations handle hardware specifics (helps testing, replay, and sim).
- Logging & replay: AdvantageKit is integrated (see `replayWatch` Gradle task and vendordeps). Logs and deterministic playback are part of the development flow.

Build / test / run (Windows PowerShell)
- Format & build (Spotless enforced during compile):
  - `.\\gradlew.bat build` — runs formatting, compiles, and runs tests.
  - `.\\gradlew.bat test` — runs unit tests (JUnit 5 configured in `build.gradle`).
- Deploy to RoboRIO: `.\\gradlew.bat deploy` (uses `deploy` target in `build.gradle`).
- Run the desktop simulation / robot main: `.\\gradlew.bat run` (GradleRIO provides desktop/run tasks when desktop support is enabled).
- Replay logs / AdvantageKit replay watch: `.\\gradlew.bat replayWatch` (task defined in `build.gradle`).

Project-specific conventions and patterns
- Naming: Subsystems and Commands use UpperCamelCase (e.g., `DriveSubsystem`, `RunShooterAtRPM`). Constants inside constants files use `kUpperCamelCase` (e.g., `kMaxSpeed`); elsewhere `UPPER_SNAKE_CASE`.
- IO pattern: Always prefer adding a new `XxxIO` interface when introducing hardware interactions. Provide at least a real and a sim implementation where feasible.
- Versioning & generated BuildConstants: `gversion` produces `BuildConstants` (configured in `build.gradle`) — avoid hand-editing generated version files.
- Vendordeps: library versions are read from `vendordeps/*.json`. Use those to determine expected API versions.

Integration points / notable external deps
- PathPlanner: route files under `src/main/deploy/pathplanner/` and `src/main/deploy/autos/` (packaged to deploy). Look at `pathplanner` settings when modifying autonomous routines.
- Vision: `VisionIOLimelight.java` and `VisionIOPhotonVisionSim.java` show how Limelight/PhotonVision are wrapped by IO interfaces.
- Motor controllers: CTRE Phoenix 6 integration is present (see `Phoenix6.json` under `vendordeps` and `PhoenixUtil.java` in `util/`).

How to modify hardware code safely
- Add or change an `XxxIO` interface and implement a real and sim IO class. Update DI/constructor wiring in the subsystem to select implementation by environment (sim vs real).
- Keep pure logic in subsystem methods and avoid direct hardware calls outside IO implementations.

Checks and quick fixes an AI may do
- Run `gradlew build` locally to catch Spotless failures — the project enforces formatting during compile.
- When changing robot behavior, update or add corresponding `src/main/deploy` pathplanner or config JSONs if autonomous behavior changed.
- Use vendordeps JSON to pick compatible library versions for any new dependency.

Where to look for examples
- `src/main/java/frc/robot/subsystems/intake/` — full IO / real / sim pattern.
- `src/main/java/frc/robot/subsystems/vision/` — vision IO variations and observation objects.
- `build.gradle` — GradleRIO, Spotless, replayWatch task, and packaging rules.

If something is unclear or missing
- Ask for the intended runtime (robot vs sim vs replay) and which hardware (motor vendors / sensors) are being targeted. Also confirm whether changes should update `src/main/deploy` assets (pathplanner/autos).

Please review this draft and tell me any missing specifics (CI commands, custom scripts, or team preferences) you want included.
