// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HoodCommands;
import frc.robot.commands.TurretCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.simulation.VisualizeFuelShot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.serializer.SerializerIO;
import frc.robot.subsystems.serializer.SerializerIOSim;
import frc.robot.subsystems.serializer.SerializerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  // private final Vision vision;
  private final Turret turret;
  private final Shooter shooter;
  private final Hood hood;
  private final Intake intake;
  private final Serializer serializer;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));

        // vision = new Vision(new VisionIOLimelight());
        // vision.setDrive(drive);
        turret = new Turret(new TurretIOTalonFX());

        shooter = new Shooter(new ShooterIOTalonFX());

        hood = new Hood(new HoodIOTalonFX());

        intake = new Intake(new IntakeIOTalonFX());

        serializer = new Serializer(new SerializerIOTalonFX());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        // vision = new Vision(new VisionIOPhotonVisionSim());
        // vision.setDrive(drive);
        turret = new Turret(new TurretIOSim());
        shooter = new Shooter(new ShooterIOSim());
        hood = new Hood(new HoodIOSim());
        intake = new Intake(new IntakeIOSim());
        serializer = new Serializer(new SerializerIOSim());

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        // vision = new Vision(new VisionIO() {});
        // vision.setDrive(drive);
        turret = new Turret(new TurretIO() {});

        shooter = new Shooter(new ShooterIO() {});
        hood = new Hood(new HoodIO() {});
        intake = new Intake(new IntakeIO() {});
        serializer = new Serializer(new SerializerIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // Default turret manual control on right stick X
    //   turret.setDefaultCommand(TurretCommands.pointAtHub(turret));

    //  operator.start().onTrue(Commands.runOnce(() -> turret.seed(), turret));
    // turret.setDefaultCommand(TurretCommands.joystickTurret(turret, () -> -operator.getRightX()));

    driver
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () ->
                    CommandScheduler.getInstance()
                        .schedule(VisualizeFuelShot.visualizeFuelShot())));

    // Lock to 0 when A button is held
    // driver
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () ->
    // Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    // driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0 when B button is pressed
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Reset turret to zero when Y pressed
    // driver.y().onTrue(Commands.runOnce(() -> turret.setAngle(Radians.of(0)), turret));

    turret.setDefaultCommand(TurretCommands.joystickTurret(turret, () -> operator.getLeftX()));
    // intake.setDefaultCommand(IntakeCommands.joystickWrist(intake, () -> operator.getLeftX()));
    // shooter.setDefaultCommand(ShooterCommands.joystickShooter(shooter, () ->
    // operator.getLeftY()));
    hood.setDefaultCommand(HoodCommands.joystickHood(hood, () -> operator.getLeftY()));

    // ==================== OPERATOR BUTTON BINDINGS ====================

    // A button: Intake down (0°)
    //  operator.a().onTrue(IntakeCommands.setWristAngle(intake, Degrees.of(0.0)));

    // B button: Intake up (120°)
    // operator.b().onTrue(IntakeCommands.setWristAngle(intake, Degrees.of(120.0)));

    operator.x().whileTrue(TurretCommands.toTurretPosition(turret, Degrees.of(-50)));
    operator.y().whileTrue(TurretCommands.toTurretPosition(turret, Degrees.of(200)));
    // Y button: Auto-aim turret towards hub
    // TODO: Implement after auto-aim logic is finalized
    // operator.y().whileTrue(TurretCommands.pointAtHub(turret));

    // X button: Toggle serializer and feeder motors on/off
    /*  operator
    .x()
    .onTrue(
        Commands.startEnd(
            () -> {
              serializer.setRPM(2000); // Serializer wheel
              serializer.setFeederRPM(2000); // Feeder rollers
            },
            () -> {
              serializer.stop();
              serializer.stopFeeder();
            },
            serializer));*/

    // Right trigger: Shoot balls (hold to shoot, release to stop)
    operator
        .rightTrigger()
        .onTrue(
            Commands.run(
                () -> shooter.setOpenLoop(10.0), // 10V while held
                shooter))
        .onFalse(Commands.runOnce(shooter::stop, shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Gets the drive subsystem. Used for simulation updates.
   *
   * @return the drive subsystem
   */
  public Drive getDrive() {
    return drive;
  }
}
