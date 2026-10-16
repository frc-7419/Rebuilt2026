package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAim;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.SerializerCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/**
 * Central manager for control state (intaking, shooting, auto-aim mode, hub mode). Updates {@link
 * RobotState} so that teleop and autonomous use the same state uniformly.
 */
public final class ControlManager {

  // --------------- Constants ---------------
  private static final double kShootSerializerVolts = 12;
  private static final double kShootFeederVolts = 11.0;
  private static final double kIntakeVolts = 8.0;
  /** Matches {@link frc.robot.commands.DriveCommands} joystick deadband. */
  private static final double kDriveJoystickDeadband = 0.1;

  private static final double kShootTurretAlignToleranceDeg = 10.0;

  /** Stowed wrist. The wheel must not run here. */
  private static final double kIntakeUpDeg = 0.0;
  /**
   * Deployed wrist. Matches {@link frc.robot.subsystems.intake.IntakeConstants#kMaxWristAngle}. The
   * wheel runs only once {@link RobotState#isIntakeDown()} is true.
   */
  private static final double kIntakeDownDeg = 120.0;
  /** Same profile as the operator wiggle on pov-left. */
  private static final double kWiggleMidpointDeg = 80.0;

  private static final double kWiggleAmplitudeDeg = 60.0;
  /** Shoot button must be held this long before the wrist wiggle starts. */
  private static final double kShootWiggleDelaySec = 2.0;

  // --------------- Singleton ---------------
  private static ControlManager instance;

  public static ControlManager getInstance() {
    if (instance == null) {
      instance = new ControlManager();
    }
    return instance;
  }

  // --------------- State ---------------
  private final RobotState robotState = RobotState.getInstance();

  /** FPGA time when the shoot button last went down. NaN while it is released. */
  private double shootButtonPressedAtSec = Double.NaN;

  /** Previous cycle's {@link RobotState#isShooting()}, so the wrist can drop when a shot ends. */
  private boolean wasShooting = false;

  /** True after this shot hold has wiggled, so the wrist drops when the button is released. */
  private boolean wiggledThisShot = false;

  private ControlManager() {}

  /** Same mapping as {@link frc.robot.commands.DriveCommands} (package-local there). */
  private static Translation2d linearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), kDriveJoystickDeadband);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    linearMagnitude = linearMagnitude * linearMagnitude;
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Teleop drive: field-relative. Kept here so {@link frc.robot.commands.DriveCommands} stays
   * unchanged.
   */
  private Command driverTeleopDrive(Drive drive, CommandXboxController driver) {
    return Commands.run(
        () -> {
          Translation2d linearVelocity =
              linearVelocityFromJoysticks(-driver.getLeftY(), -driver.getLeftX());
          double omega = MathUtil.applyDeadband(-driver.getRightX(), kDriveJoystickDeadband);
          omega = Math.copySign(omega * omega, omega);
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  // --------------- Intake ---------------

  private static boolean forwardIntakeHeld(
      CommandXboxController driver, CommandXboxController operator) {
    return driver.leftTrigger().getAsBoolean() || operator.leftTrigger().getAsBoolean();
  }

  private static boolean reverseIntakeHeld(
      CommandXboxController driver, CommandXboxController operator) {
    return driver.leftBumper().getAsBoolean() || operator.leftBumper().getAsBoolean();
  }

  private static boolean intakeRequested(
      CommandXboxController driver, CommandXboxController operator) {
    return forwardIntakeHeld(driver, operator) || reverseIntakeHeld(driver, operator);
  }

  private static boolean shootButtonHeld(
      CommandXboxController driver, CommandXboxController operator) {
    return driver.rightTrigger().getAsBoolean() || operator.rightTrigger().getAsBoolean();
  }

  /** Manual wrist buttons and their release actions stay idle while intake or shooting owns it. */
  private boolean manualWristIdle(CommandXboxController driver, CommandXboxController operator) {
    return !intakeRequested(driver, operator)
        && !robotState.isIntaking()
        && !robotState.isShooting();
  }

  /**
   * Tracks how long the shoot button has been held. Wiggle is allowed only after {@link
   * #kShootWiggleDelaySec} and while {@link RobotState#isShooting()} is true.
   */
  private boolean shootWiggleReady(CommandXboxController driver, CommandXboxController operator) {
    if (!shootButtonHeld(driver, operator)) {
      shootButtonPressedAtSec = Double.NaN;
      return false;
    }
    double now = Timer.getFPGATimestamp();
    if (Double.isNaN(shootButtonPressedAtSec)) {
      shootButtonPressedAtSec = now;
    }
    return robotState.isShooting() && (now - shootButtonPressedAtSec) >= kShootWiggleDelaySec;
  }

  /** Commands the deployed wrist angle. Does not run the wheel. */
  private void holdIntakeDown(Intake intake) {
    intake.setWristAngle(Degrees.of(kIntakeDownDeg));
  }

  /**
   * Holds the wrist down and runs the wheel only after the measured angle is down. {@link
   * RobotState#isIntaking()} stays false while the wrist is still up.
   */
  private void runWheelIfDown(Intake intake, double volts) {
    holdIntakeDown(intake);
    if (robotState.isIntakeDown()) {
      robotState.setIntaking(true);
      intake.setWheelOpenLoop(volts);
    } else {
      robotState.setIntaking(false);
      intake.stopWheel();
    }
  }

  /**
   * Lowers the wrist and runs the wheel once it is down. Requires Intake and refuses incoming
   * commands so a manual wrist binding cannot cancel an in-progress intake.
   */
  public Command runIntakeWheel(Intake intake, double volts) {
    return Commands.run(() -> runWheelIfDown(intake, volts), intake)
        .finallyDo(
            interrupted -> {
              robotState.setIntaking(false);
              intake.stopWheel();
            })
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  /**
   * Single Intake owner for teleop and for shooting outside an auto intake command.
   *
   * <p>Priority: trigger intake (wrist down, wheel only when down), then shoot wiggle after the
   * shoot button has been held for {@link #kShootWiggleDelaySec}. Releasing that shot commands the
   * wrist down. Idle does not clear a stow or deploy angle.
   */
  private Command intakeModeCommand(
      Intake intake, CommandXboxController driver, CommandXboxController operator) {
    return Commands.run(
            () -> {
              boolean forward = forwardIntakeHeld(driver, operator);
              boolean reverse = reverseIntakeHeld(driver, operator);
              boolean shooting = robotState.isShooting();
              boolean wiggle = shootWiggleReady(driver, operator);
              if (forward || reverse) {
                if (forward && reverse) {
                  holdIntakeDown(intake);
                  robotState.setIntaking(false);
                  intake.stopWheel();
                } else {
                  runWheelIfDown(intake, forward ? kIntakeVolts : -kIntakeVolts);
                }
              } else if (wiggle) {
                wiggledThisShot = true;
                robotState.setIntaking(false);
                intake.stopWheel();
                IntakeCommands.applyWristWiggle(
                    intake, Degrees.of(kWiggleMidpointDeg), Degrees.of(kWiggleAmplitudeDeg));
              } else if (!shooting && wasShooting && wiggledThisShot) {
                robotState.setIntaking(false);
                intake.stopWheel();
                holdIntakeDown(intake);
              } else {
                robotState.setIntaking(false);
                intake.stopWheel();
              }
              if (!shooting) {
                wiggledThisShot = false;
              }
              wasShooting = shooting;
            },
            intake)
        .finallyDo(
            interrupted -> {
              robotState.setIntaking(false);
              intake.stopWheel();
            });
  }

  /**
   * Lowers the wrist and runs the intake wheel once it is down. One command so PathPlanner does not
   * schedule a second Intake requirement. Incoming commands cannot cancel it.
   */
  public Command runIntakeLowerAndWheel(Intake intake) {
    return runIntakeWheel(intake, kIntakeVolts);
  }

  public Command stopIntake(Intake intake) {
    return Commands.runOnce(
        () -> {
          robotState.setIntaking(false);
          intake.stopWheel();
        },
        intake);
  }

  // --------------- Shooting ---------------

  public Command stopShooting(Serializer serializer) {
    return Commands.runOnce(
        () -> {
          robotState.setShooting(false);
          serializer.stopBoth();
        },
        serializer);
  }

  public void toggleAutoAim() {
    robotState.setAutoAimEnabled(!robotState.isAutoAimEnabled());
  }

  public void setAutoAimEnabled(boolean enabled) {
    robotState.setAutoAimEnabled(enabled);
  }

  public void toggleHubMode() {
    robotState.setHubMode(!robotState.isHubMode());
  }

  public void setHubMode(boolean hubMode) {
    robotState.setHubMode(hubMode);
  }

  public boolean isAutoAimEnabled() {
    return robotState.isAutoAimEnabled();
  }

  public boolean isHubMode() {
    return robotState.isHubMode();
  }

  /**
   * Runs serializer/feeder only when shooter RPM is within {@link
   * ShooterConstants#kRpmToleranceForReady} of the request and the turret is within {@link
   * #kShootTurretAlignToleranceDeg} of its setpoint (see {@link
   * RobotState#isTurretWithinSetpoint}).
   */
  public Command runShootWhenReady(Shooter shooter, Serializer serializer) {
    return buildShootWhenReadyCommand(shooter, serializer, true);
  }

  /**
   * Same as {@link #runShootWhenReady} but with no subsystem requirements (for PathPlanner event
   * zones that conflict with requirement scheduling).
   */
  public Command runShootWhenReadyNoRequirements(Shooter shooter, Serializer serializer) {
    return buildShootWhenReadyCommand(shooter, serializer, false);
  }

  private Command buildShootWhenReadyCommand(
      Shooter shooter, Serializer serializer, boolean requireSerializer) {
    Runnable run =
        () -> {
          double current = shooter.getRotorRPM();
          double requested = shooter.getRequestedRPM();
          boolean shooterReady = (requested - current) <= ShooterConstants.kRpmToleranceForReady;
          Logger.recordOutput("Shooter/Ready", shooterReady);
          boolean turretReady =
              robotState.isTurretWithinSetpoint(Degrees.of(kShootTurretAlignToleranceDeg));
          Logger.recordOutput("Shooter/TurretReady", turretReady);
          if (shooterReady && turretReady) {
            serializer.setFeederVoltage(kShootFeederVolts);
            serializer.setSerializerVoltage(kShootSerializerVolts);
          } else {
            serializer.stopBoth();
          }
        };
    Command base = requireSerializer ? Commands.run(run, serializer) : Commands.run(run);
    return base.beforeStarting(Commands.runOnce(() -> robotState.setShooting(true)))
        .finallyDo(
            interrupted -> {
              robotState.setShooting(false);
              serializer.stopBoth();
            })
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  /**
   * Registers named commands for PathPlanner/autonomous. Uses the same {@link RobotState} as teleop
   * so auto and teleop share auto-aim and hub mode state.
   */
  public void registerNamedCommands(Intake intake, Shooter shooter, Serializer serializer) {
    Command wiggle = IntakeCommands.setWristAngleWiggle(intake, Degrees.of(70), Degrees.of(60), 7);
    NamedCommands.registerCommand("EnableAutoAim", Commands.runOnce(() -> setAutoAimEnabled(true)));
    NamedCommands.registerCommand(
        "DisableAutoAim", Commands.runOnce(() -> setAutoAimEnabled(false)));
    NamedCommands.registerCommand("EnableHubMode", Commands.runOnce(() -> setHubMode(true)));
    NamedCommands.registerCommand("DisableHubMode", Commands.runOnce(() -> setHubMode(false)));
    NamedCommands.registerCommand("Intake", runIntakeLowerAndWheel(intake));
    NamedCommands.registerCommand("AutoShoot", runShootWhenReady(shooter, serializer));
    NamedCommands.registerCommand(
        "LowerIntake", IntakeCommands.setWristAngle(intake, Degrees.of(kIntakeDownDeg)));
    NamedCommands.registerCommand(
        "RaiseIntake", IntakeCommands.setWristAngle(intake, Degrees.of(kIntakeUpDeg)));
    NamedCommands.registerCommand("WiggleIntake", wiggle);

    new EventTrigger("Intake").whileTrue(runIntakeLowerAndWheel(intake));
    new EventTrigger("WiggleIntake").whileTrue(wiggle);
    new EventTrigger("AutoShoot").whileTrue(runShootWhenReadyNoRequirements(shooter, serializer));
  }

  /** Configures driver and operator button bindings. */
  public void configureButtonBindings(
      CommandXboxController driver,
      CommandXboxController operator,
      Drive drive,
      Turret turret,
      Shooter shooter,
      Hood hood,
      Intake intake,
      Vision vision,
      Serializer serializer) {
    // Driver triggers. Reset pose is Start so it is not next to the intake bumper.
    Trigger stopSwerveTrigger = driver.x();
    Trigger lockPushOrientationTrigger = driver.a();
    Trigger resetPoseTrigger = driver.start();

    // Operator triggers: mode toggles. Operator Start stays hub mode.
    Trigger hubModeTrigger = operator.start();
    Trigger autoAimTrigger = operator.x();

    // Both controllers: left trigger intake, left bumper reverse intake, right trigger shoot,
    // right bumper reverse serializer. Manual wrist stays off while intake or shooting owns it.
    Trigger intakeRequestedTrigger =
        driver
            .leftTrigger()
            .or(operator.leftTrigger())
            .or(driver.leftBumper())
            .or(operator.leftBumper());
    Trigger shootTrigger = driver.rightTrigger().or(operator.rightTrigger());
    Trigger shootingTrigger = new Trigger(robotState::isShooting);
    Trigger manualWristAllowed =
        intakeRequestedTrigger
            .negate()
            .and(new Trigger(robotState::isIntaking).negate())
            .and(shootingTrigger.negate());
    // Right bumper must not steal Serializer from a shot, or its release stop would cancel it.
    Trigger reverseSerializerTrigger =
        driver
            .rightBumper()
            .or(operator.rightBumper())
            .and(shootTrigger.negate())
            .and(shootingTrigger.negate());

    // Operator triggers: wrist
    Trigger intakeJerkingTrigger = operator.povLeft().and(manualWristAllowed);
    Trigger wristStowTrigger = operator.b().and(manualWristAllowed);
    Trigger wristDeployTrigger = operator.a().and(manualWristAllowed);
    Trigger wristUpTrigger = operator.povUp().and(manualWristAllowed);
    Trigger wristDownTrigger = operator.povDown().and(manualWristAllowed);

    // Operator triggers: shooter. Auto-aim already writes shooter RPM from the turret command.
    Trigger revShooterTrigger =
        operator.y().and(new Trigger(robotState::isAutoAimEnabled).negate());

    // -------- Driver: drive & pose --------
    drive.setDefaultCommand(driverTeleopDrive(drive, driver));

    lockPushOrientationTrigger.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> Rotation2d.fromDegrees(90)));

    stopSwerveTrigger.onTrue(Commands.runOnce(drive::stopWithX, drive));

    resetPoseTrigger.onTrue(
        Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive)
            .ignoringDisable(true));

    // -------- Operator: turret & hood (default commands) --------
    turret.setDefaultCommand(
        Commands.run(
            () -> {
              if (robotState.isAutoAimEnabled()) {
                boolean allowFullRange = robotState.isShooting();
                boolean effectiveHubMode =
                    AutoAim.useHubMode(
                        robotState.isHubMode(), drive.getPose(), robotState.isRedAlliance());
                AutoAim.updateAutoAim(turret, shooter, hood, effectiveHubMode, allowFullRange);
              } else {
                double v = MathUtil.applyDeadband(operator.getRightX(), 0.05);
                turret.setOpenLoop(v * 12.0);
              }
            },
            turret));

    hood.setDefaultCommand(
        Commands.run(
            () -> {
              if (!robotState.isAutoAimEnabled()) {
                double v = MathUtil.applyDeadband(operator.getLeftY(), 0.05);
                hood.setOpenLoop(v * 3.0);
              }
            },
            hood));

    // -------- Operator: mode toggles --------
    hubModeTrigger.onTrue(Commands.runOnce(this::toggleHubMode));
    autoAimTrigger.onTrue(Commands.runOnce(this::toggleAutoAim));

    // -------- Operator: intake & shooting --------
    // Default command restarts after a manual wrist command ends. A whileTrue on the bumpers
    // would not, so intake lives here instead of on its own trigger.
    intake.setDefaultCommand(intakeModeCommand(intake, driver, operator));
    shootTrigger.whileTrue(runShootWhenReady(shooter, serializer));

    intakeJerkingTrigger
        .whileTrue(
            IntakeCommands.setWristAngleWiggle(
                intake, Degrees.of(kWiggleMidpointDeg), Degrees.of(kWiggleAmplitudeDeg), 0.0))
        .onFalse(
            Commands.runOnce(
                () -> {
                  if (manualWristIdle(driver, operator)) {
                    intake.setWristAngle(Degrees.of(kIntakeDownDeg));
                  }
                }));

    reverseSerializerTrigger
        .whileTrue(
            SerializerCommands.runBothVoltage(
                serializer, -kShootSerializerVolts, -kShootFeederVolts))
        .onFalse(
            Commands.runOnce(
                () -> {
                  if (!shootTrigger.getAsBoolean() && !robotState.isShooting()) {
                    serializer.stopBoth();
                  }
                }));

    // -------- Operator: wrist --------
    wristStowTrigger.onTrue(
        Commands.runOnce(() -> intake.setWristAngle(Degrees.of(kIntakeUpDeg)), intake));
    wristDeployTrigger.onTrue(
        Commands.runOnce(() -> intake.setWristAngle(Degrees.of(kIntakeDownDeg)), intake));

    wristUpTrigger
        .whileTrue(Commands.run(() -> intake.setWristOpenLoop(-2.0), intake))
        .onFalse(
            Commands.runOnce(
                () -> {
                  if (manualWristIdle(driver, operator)) {
                    intake.stopWrist();
                  }
                }));

    wristDownTrigger
        .whileTrue(Commands.run(() -> intake.setWristOpenLoop(2.0), intake))
        .onFalse(
            Commands.runOnce(
                () -> {
                  if (manualWristIdle(driver, operator)) {
                    intake.stopWrist();
                  }
                }));

    // -------- Operator: shooter --------
    revShooterTrigger
        .whileTrue(Commands.run(() -> shooter.setVelocity(RPM.of(2500)), shooter))
        .onFalse(
            Commands.runOnce(
                () -> {
                  if (!robotState.isAutoAimEnabled()) {
                    shooter.setVelocity(RPM.of(0));
                  }
                }));
  }
}
