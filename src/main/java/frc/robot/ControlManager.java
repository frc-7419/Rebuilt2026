package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
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

/**
 * Central manager for control state (intaking, shooting, auto-aim mode, hub mode). Updates {@link
 * RobotState} so that teleop and autonomous use the same state uniformly.
 */
public final class ControlManager {

  // --------------- Constants ---------------
  private static final double kShootSerializerVolts = 10.0;
  private static final double kShootFeederVolts = 10.0;
  private static final double kIntakeVolts = 6;

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
  private double lastWristAngleDeg = 0.0;

  private ControlManager() {}

  // --------------- Intake ---------------

  /** Runs the intake wheel and sets {@link RobotState#setIntaking(boolean)} */
  public Command runIntakeWheel(Intake intake, double volts) {
    return Commands.run(() -> intake.setWheelOpenLoop(volts), intake)
        .beforeStarting(Commands.runOnce(() -> robotState.setIntaking(true)))
        .finallyDo(
            interrupted -> {
              robotState.setIntaking(false);
              intake.stopWheel();
            });
  }

  /**
   * Lowers wrist to 0° and runs the intake wheel; single command so it only requires Intake once.
   */
  public Command runIntakeLowerAndWheel(Intake intake) {
    return Commands.run(
            () -> {
              intake.setWristAngle(Degrees.of(120.0));
              intake.setWheelOpenLoop(kIntakeVolts);
            },
            intake)
        .beforeStarting(Commands.runOnce(() -> robotState.setIntaking(true)))
        .finallyDo(
            interrupted -> {
              robotState.setIntaking(false);
              intake.stopWheel();
            });
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
   * Runs serializer/feeder for shooting only when shooter is within {@link
   * ShooterConstants#kRpmToleranceForReady} RPM of the requested speed.
   */
  public Command runShootAtSpeed(Shooter shooter, Serializer serializer) {
    return buildShootAtSpeedCommand(shooter, serializer, true);
  }

  /**
   * Same as {@link #runShootAtSpeed} but with no subsystem requirements (for PathPlanner event
   * zones that conflict with requirement scheduling).
   */
  public Command runShootAtSpeedNoRequirements(Shooter shooter, Serializer serializer) {
    return buildShootAtSpeedCommand(shooter, serializer, false);
  }

  private Command buildShootAtSpeedCommand(
      Shooter shooter, Serializer serializer, boolean requireSerializer) {
    Runnable run =
        () -> {
          double current = shooter.getRotorRPM();
          double requested = shooter.getRequestedRPM();
          if (Math.abs(current - requested) <= ShooterConstants.kRpmToleranceForReady) {
            serializer.setFeederVoltage(kShootFeederVolts);
            double t = Timer.getFPGATimestamp();
            serializer.setSerializerVoltage((t - (int) t) < 0.7 ? kShootSerializerVolts : 0);
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
            });
  }

  /**
   * Registers named commands for PathPlanner/autonomous. Uses the same {@link RobotState} as teleop
   * so auto and teleop share auto-aim and hub mode state.
   */
  public void registerNamedCommands(Intake intake, Shooter shooter, Serializer serializer) {
    Command wiggle = IntakeCommands.setWristAngleWiggle(intake, Degrees.of(70), Degrees.of(20), 7);
    NamedCommands.registerCommand("EnableAutoAim", Commands.runOnce(() -> setAutoAimEnabled(true)));
    NamedCommands.registerCommand(
        "DisableAutoAim", Commands.runOnce(() -> setAutoAimEnabled(false)));
    NamedCommands.registerCommand("EnableHubMode", Commands.runOnce(() -> setHubMode(true)));
    NamedCommands.registerCommand("DisableHubMode", Commands.runOnce(() -> setHubMode(false)));
    NamedCommands.registerCommand("Intake", runIntakeLowerAndWheel(intake));
    NamedCommands.registerCommand("AutoShoot", runShootAtSpeed(shooter, serializer));
    NamedCommands.registerCommand(
        "LowerIntake", IntakeCommands.setWristAngle(intake, Degrees.of(120.0)));
    NamedCommands.registerCommand(
        "RaiseIntake", IntakeCommands.setWristAngle(intake, Degrees.of(0.0)));
    NamedCommands.registerCommand("WiggleIntake", wiggle);

    new EventTrigger("Intake").whileTrue(runIntakeLowerAndWheel(intake));
    new EventTrigger("WiggleIntake").whileTrue(wiggle);
    new EventTrigger("AutoShoot").whileTrue(runShootAtSpeedNoRequirements(shooter, serializer));
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
      Serializer serializer) {
    // Driver triggers
    Trigger stopSwerveTrigger = driver.x();
    Trigger lockPushOrientationTrigger = driver.a();
    Trigger resetPoseTrigger = driver.leftBumper();

    // Operator triggers: mode toggles
    Trigger hubModeTrigger = operator.start();
    Trigger autoAimTrigger = operator.x();

    // Operator triggers: intake & shooting
    Trigger intakeWheelTrigger = operator.rightBumper();
    Trigger reverseIntakeTrigger = operator.leftBumper();
    Trigger shootTrigger = operator.rightTrigger();
    Trigger reverseSerializerTrigger = operator.leftTrigger();
    Trigger intakeJerkingTrigger = operator.povLeft();

    // Operator triggers: wrist
    Trigger wristStowTrigger = operator.b();
    Trigger wristDeployTrigger = operator.a();
    Trigger wristUpTrigger = operator.povUp();
    Trigger wristDownTrigger = operator.povDown();

    // Operator triggers: shooter
    Trigger revShooterTrigger = operator.y();

    // -------- Driver: drive & pose --------
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

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
                AutoAim.updateAutoAim(
                    turret, shooter, hood, robotState.isHubMode(), allowFullRange);
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
    intakeWheelTrigger.whileTrue(runIntakeWheel(intake, kIntakeVolts));
    reverseIntakeTrigger.whileTrue(runIntakeWheel(intake, -kIntakeVolts));
    shootTrigger.whileTrue(runShootAtSpeed(shooter, serializer));

    intakeJerkingTrigger
        .whileTrue(
            IntakeCommands.setWristAngleWiggle(
                intake, Degrees.of(70), Degrees.of(20), kIntakeVolts))
        .onFalse(
            Commands.runOnce(() -> intake.setWristAngle(Degrees.of(lastWristAngleDeg)), intake));

    reverseSerializerTrigger
        .whileTrue(
            SerializerCommands.runBothVoltage(
                serializer, -kShootSerializerVolts, -kShootFeederVolts))
        .onFalse(SerializerCommands.stopBoth(serializer));

    // -------- Operator: wrist --------
    wristStowTrigger.onTrue(
        Commands.runOnce(
            () -> {
              lastWristAngleDeg = 0.0;
              intake.setWristAngle(Degrees.of(0.0));
            },
            intake));
    wristDeployTrigger.onTrue(
        Commands.runOnce(
            () -> {
              lastWristAngleDeg = 120.0;
              intake.setWristAngle(Degrees.of(120.0));
            },
            intake));

    wristUpTrigger
        .whileTrue(Commands.run(() -> intake.setWristOpenLoop(-2.0), intake))
        .onFalse(Commands.runOnce(intake::stopWrist, intake));

    wristDownTrigger
        .whileTrue(Commands.run(() -> intake.setWristOpenLoop(2.0), intake))
        .onFalse(Commands.runOnce(intake::stopWrist, intake));

    // -------- Operator: shooter --------
    revShooterTrigger
        .whileTrue(Commands.run(() -> shooter.setVelocity(RPM.of(4000)), shooter))
        .onFalse(Commands.runOnce(() -> shooter.setVelocity(RPM.of(0)), shooter));
  }
}
