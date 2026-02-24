package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

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

/**
 * Central manager for control state (intaking, shooting, auto-aim mode, hub mode). Updates {@link
 * RobotState} so that teleop and autonomous use the same state uniformly.
 */
public final class ControlManager {

  private static final double kShootSerializerVolts = 5.0;
  private static final double kShootFeederVolts = 7.0;
  private static final double kIntakeVolts = 3;

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
  public Command runShooting(Serializer serializer) {
    return Commands.run(
            () -> serializer.setBothVoltage(kShootSerializerVolts, kShootFeederVolts), serializer)
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

    Trigger stopSwerveTrigger = driver.x();
    Trigger resetPoseTrigger = driver.leftBumper();
    Trigger hubModeTrigger = operator.start();
    Trigger autoAimTrigger = operator.x();
    Trigger intakeWheelTrigger = operator.leftBumper();
    Trigger shootTrigger = operator.rightTrigger();
    Trigger reverseSerializerTrigger = operator.leftTrigger();
    Trigger wristStowTrigger = operator.b();
    Trigger wristDeployTrigger = operator.a();
    Trigger wristUpTrigger = operator.povUp();
    Trigger wristDownTrigger = operator.povDown();
    Trigger revShooterTrigger = operator.rightBumper();

    // -------- Driver --------
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

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
    intakeWheelTrigger.whileTrue(runIntakeWheel(intake, kIntakeVolts));
    shootTrigger.whileTrue(runShooting(serializer));

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

    revShooterTrigger.whileTrue(Commands.run(() -> shooter.setRPM(3500), shooter));
  }
}
