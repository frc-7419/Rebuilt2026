package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.turret.Turret;
import java.util.function.DoubleSupplier;

/**
 * Central manager for control state (intaking, shooting, auto-aim mode, hub mode). Updates {@link
 * RobotState} so that teleop and autonomous use the same state uniformly.
 */
public final class ControlManager {

  private static final double kShootSerializerVolts = 10.0;
  private static final double kShootFeederVolts = 10.0;
  private static final double kIntakeVolts = 5;

  private static ControlManager instance;

  public static ControlManager getInstance() {
    if (instance == null) instance = new ControlManager();
    return instance;
  }

  private final RobotState robotState = RobotState.getInstance();

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

  /** Runs serializer/feeder for shooting and sets {@link RobotState#setShooting(boolean)} */
  public Command runShooting(Serializer serializer, DoubleSupplier rpsSupplier) {
    return Commands.run(
            () -> {
              if (Math.abs(
                      rpsSupplier.getAsDouble()
                          - RobotState.getInstance()
                              .getLatestShooterVelocity()
                              .in(RotationsPerSecond))
                  < 100) serializer.setBothVoltage(kShootSerializerVolts, kShootFeederVolts);
              else serializer.setBothVoltage(0, 0);
            },
            serializer)
        .beforeStarting(Commands.runOnce(() -> robotState.setShooting(true)))
        .finallyDo(
            interrupted -> {
              robotState.setShooting(false);
              serializer.stopBoth();
            });
  }

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

  private static final double kShootRpmTolerance = 200.0;

  /**
   * Runs serializer/feeder for shooting only when shooter is within {@value #kShootRpmTolerance}
   * RPM of the requested speed.
   */
  public Command runShootAtSpeed(Shooter shooter, Serializer serializer) {
    return Commands.run(
            () -> {
              double current = shooter.getRPM();
              double requested = shooter.getRequestedRPM();
              if (Math.abs(current - requested) <= kShootRpmTolerance) {
                serializer.setBothVoltage(kShootSerializerVolts, kShootFeederVolts);
              } else {
                serializer.stopBoth();
              }
            },
            serializer)
        .beforeStarting(Commands.runOnce(() -> robotState.setShooting(true)))
        .finallyDo(
            interrupted -> {
              robotState.setShooting(false);
              serializer.stopBoth();
            });
  }

  /**
   * Same behavior as {@link #runShootAtSpeed} but with no subsystem requirements for zoned events
   * idk it was bugging
   */
  public Command runShootAtSpeedNoRequirements(Shooter shooter, Serializer serializer) {
    return Commands.run(
            () -> {
              double current = shooter.getRPM();
              double requested = shooter.getRequestedRPM();
              if (Math.abs(current - requested) <= kShootRpmTolerance) {
                serializer.setBothVoltage(kShootSerializerVolts, kShootFeederVolts);
              } else {
                serializer.stopBoth();
              }
            })
        .beforeStarting(Commands.runOnce(() -> robotState.setShooting(true)))
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
    // New Controls
    /*
     * Trigger stopSwerveTrigger = driver.x();
     * Trigger lockPushOrientationTrigger = driver.a();
     * Trigger resetPoseTrigger = driver.leftBumper();
     * Trigger hubModeTrigger = operator.start();
     * Trigger autoAimTrigger = operator.x();
     * Trigger intakeWheelTrigger = operator.leftBumper();
     * Trigger shootTrigger = operator.rightTrigger();
     * Trigger reverseSerializerTrigger = operator.leftTrigger();
     * Trigger wristStowTrigger = operator.b();
     * Trigger wristDeployTrigger = operator.a();
     * Trigger wristUpTrigger = operator.povUp();
     * Trigger wristDownTrigger = operator.povDown();
     * Trigger revShooterTrigger = operator.rightBumper();
     * Trigger hoodPositionDownTrigger = operator.y();
     * Trigger hoodPositionUpTrigger = operator.back();
     * Trigger reverseIntakeTrigger = operator.povLeft();
     */

    Trigger stopSwerveTrigger = driver.x();
    Trigger lockPushOrientationTrigger = driver.a();
    Trigger resetPoseTrigger = driver.leftBumper();

    Trigger reverseSerializerTrigger = operator.leftTrigger();
    Trigger shootTrigger = operator.rightTrigger();
    Trigger reverseIntakeTrigger = operator.leftBumper();
    Trigger intakeWheelTrigger = operator.rightBumper();

    Trigger wristUpTrigger = operator.povUp();
    Trigger wristDownTrigger = operator.povDown();

    Trigger revShooterTrigger = operator.y();
    Trigger autoAimTrigger = operator.x();
    Trigger wristStowTrigger = operator.b();
    Trigger wristDeployTrigger = operator.a();

    Trigger hubModeTrigger = operator.start();
    Trigger intakeJerkingTrigger = operator.povLeft();

    // -------- Driver --------
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
    // intakeWheelTrigger.whileTrue(runIntakeWheel(intake, kIntakeVolts));
    intakeWheelTrigger.whileTrue(runIntakeWheel(intake, kIntakeVolts));
    reverseIntakeTrigger.whileTrue(runIntakeWheel(intake, -kIntakeVolts));
    shootTrigger.whileTrue(runShooting(serializer, () -> shooter.getRPM() / 60));

    intakeJerkingTrigger.whileTrue(
        IntakeCommands.setWristAngleWiggle(intake, Degrees.of(70), Degrees.of(20), kIntakeVolts));

    reverseSerializerTrigger
        .whileTrue(
            SerializerCommands.runBothVoltage(
                serializer, -kShootSerializerVolts, -kShootFeederVolts))
        .onFalse(SerializerCommands.stopBoth(serializer));

    wristStowTrigger.onTrue(IntakeCommands.setWristAngle(intake, Degrees.of(0.0)));
    wristDeployTrigger.onTrue(IntakeCommands.setWristAngle(intake, Degrees.of(120.0)));

    wristUpTrigger
        .whileTrue(Commands.run(() -> intake.setWristOpenLoop(-2.0), intake))
        .onFalse(Commands.runOnce(intake::stopWrist, intake));

    wristDownTrigger
        .whileTrue(Commands.run(() -> intake.setWristOpenLoop(2.0), intake))
        .onFalse(Commands.runOnce(intake::stopWrist, intake));

    revShooterTrigger
        .whileTrue(Commands.run(() -> shooter.setVelocity(RPM.of(1700)), shooter))
        .onFalse(Commands.runOnce(() -> shooter.setVelocity(RPM.of(0)), shooter));
  }
}
