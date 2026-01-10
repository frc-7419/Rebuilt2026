# FRC Team 7419 – 2026 Robot Code

## Overview

This repository contains the codebase for FRC Team 7419’s 2026 robot.

For the 2026 season, we are using **AdvantageKit** as our primary robot framework to support structured code, deterministic logging, simulation, and replay.

## Development Guidelines

### Code Quality

- **Consistency**: Adhere to established coding standards to ensure uniformity across the codebase.
- **Readability**: Write clear and understandable code to facilitate maintenance and collaboration.
- **Documentation**: Include comments where necessary to explain complex logic or decisions.

### AdvantageKit Notes

- AdvantageKit introduces **IO interfaces** to separate hardware access from subsystem logic.
- This allows us to support:
  - Real / Sim / Replay implementations
  - Cleaner testing and debugging
  - Deterministic log playback
- The learning curve is **not steep** if you are already comfortable with WPILib — most robot logic remains the same.

---

## Naming Conventions

- **Subsystems**: Use `UpperCamelCase`.  
  Example: `DriveSubsystem`

- **Commands**: Use descriptive `UpperCamelCase`.  
  Example: `RunShooterAtRPM`, `DriveToPose`

- **Constants**:
  - In constants files: Prefix with `k` and use `UpperCamelCase`.  
    Example: `kMaxSpeed`
  - Within other files: Use `UPPER_SNAKE_CASE`.  
    Example: `MAX_SPEED`

Following these conventions enhances code clarity and maintainability.

---

## Build Process

We utilize [Spotless](https://github.com/diffplug/spotless) to maintain consistent code formatting.

- Spotless checks are applied during the build process.
- **Always build the project successfully before committing** to ensure formatting compliance.

Your PR will fail if not :(

---

## Branching Strategy

- **Branch Creation**: Create a new branch for each feature or bug fix. Do not commit directly to `main`.
- **Naming**: Use clear, descriptive branch names.  
  Examples: `swerve`, `vision`, `shooter`
- **Unfinished Work**: It is acceptable to commit unfinished work to your own branch.  
  Communicate status clearly (Slack / Canvas) so others can continue if needed.

This keeps the repository organized and collaboration smooth.


## Additional Resources

- [WPILib Documentation](https://docs.wpilib.org/)
- [AdvantageKit Documentation](https://docs.advantagekit.org/)
- [CTRE Phoenix 6 Documentation](https://v6.docs.ctr-electronics.com/)
- [PathPlanner Documentation](https://pathplanner.dev/)
- [Limelight Documentation](https://docs.limelightvision.io/)
- [PhotonVision Documentation](https://docs.photonvision.org/en/latest/)
- [REVLib Documentation](https://docs.revrobotics.com/brushless/revlib/revlib-overview)
- [Elastic Dashboard](https://frcdocs.wpi.edu/en/latest/docs/software/dashboards/elastic.html)