// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoAim;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.simulation.FuelSim;
import frc.robot.simulation.FuelSimLaunch;
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
import frc.robot.util.KinematicsHelper;
import org.littletonrobotics.junction.Logger;
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

  private final RobotState robotState = RobotState.getInstance();
  private final ControlManager controlManager = ControlManager.getInstance();

  // Check pdh id and update accordingly
  private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  /** Fuel physics sim (SIM only). Null when not in SIM. */
  private FuelSim fuelSim;

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

    controlManager.registerNamedCommands(intake, shooter, serializer);

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

    controlManager.configureButtonBindings(
        driver, operator, drive, turret, shooter, hood, intake, serializer);

    if (Constants.currentMode == Constants.Mode.SIM) {
      fuelSim = new FuelSim("FuelSim");
      fuelSim.spawnStartingFuel();
      fuelSim.registerRobot(
          Meters.of(0.8749),
          Meters.of(0.8749),
          Meters.of(0.532194),
          drive::getPose,
          () -> {
            ChassisSpeeds s =
                RobotState.getInstance().getLatestMeasuredFieldRelativeChassisSpeeds();
            return s != null ? s : new ChassisSpeeds();
          });
      fuelSim.registerIntake(
          Meters.of(-0.739075),
          Meters.of(-0.43745),
          Meters.of(-0.43745),
          Meters.of(0.43745),
          () ->
              robotState.isIntaking()
                  && robotState.isIntakeDown()
                  && (!Constants.kSimulateFuelCapacity || robotState.canIntake()),
          () -> {
            if (Constants.kSimulateFuelCapacity) robotState.intakeFuel();
          });
      fuelSim.start();
      SmartDashboard.putData(
          Commands.runOnce(
                  () -> {
                    fuelSim.clearFuel();
                    fuelSim.spawnStartingFuel();
                  })
              .withName("Reset Fuel")
              .ignoringDisable(true));
      scheduleSimFuelLaunch();
    }
    pdh.setSwitchableChannel(true); // powers the limelight
  }

  private void scheduleSimFuelLaunch() {
    final double intervalSec = 0.1;
    final double minRpm = 100.0;
    final double[] lastShotTime = {Timer.getFPGATimestamp()};

    CommandScheduler.getInstance()
        .schedule(
            Commands.run(
                    () -> {
                      double now = Timer.getFPGATimestamp();
                      boolean shooterRunning =
                          shooter.getRPM() > minRpm
                              && Math.abs(serializer.getSerializerRPM()) > minRpm
                              && Math.abs(serializer.getFeederRPM()) > minRpm;
                      boolean canLaunch =
                          !Constants.kSimulateFuelCapacity || robotState.getFuelStored() > 0;
                      if (now - lastShotTime[0] >= intervalSec && shooterRunning && canLaunch) {
                        FuelSimLaunch.launchFromShooter(fuelSim, drive, shooter, hood, turret);
                        if (Constants.kSimulateFuelCapacity) robotState.consumeFuel();
                        lastShotTime[0] = now;
                      }
                    })
                .ignoringDisable(true));
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
   * Updates shooter trajectory visualization (real trajectory from current state). Call in
   * SIM/REPLAY only.
   */
  public void updateShooterTrajectoryVisualization() {
    var state = RobotState.getInstance();
    var latestPose = state.getLatestFieldToRobot();
    if (latestPose == null) return;

    Pose2d robotPose = latestPose.getValue();
    double hoodRad = hood.getAngle().in(Radians);
    double turretRad = turret.getAngle().in(Radians);
    double velConstant = AutoAim.getLaunchVelConstant();
    double launchSpeedMps = (shooter.getRPM() / 60.0) * velConstant;

    Translation2d turretPivotField = KinematicsHelper.getTurretPivotTranslation(robotPose);
    Translation2d pivotOffset = turretPivotField.minus(robotPose.getTranslation());
    var fieldSpeeds = state.getLatestMeasuredFieldRelativeChassisSpeeds();
    double robotVx = fieldSpeeds.vxMetersPerSecond;
    double robotVy = fieldSpeeds.vyMetersPerSecond;
    double robotOmega = fieldSpeeds.omegaRadiansPerSecond;
    double pivotVx = robotVx - robotOmega * pivotOffset.getY();
    double pivotVy = robotVy + robotOmega * pivotOffset.getX();

    Translation3d[] traj =
        AutoAim.buildTrajectoryFromState(
            robotPose, hoodRad, turretRad, pivotVx, pivotVy, launchSpeedMps);
    Logger.recordOutput("ShooterTrajectory/Trajectory", traj);
    Logger.recordOutput("ShooterTrajectory/LaunchSpeedMps", launchSpeedMps);
  }

  /**
   * Gets the drive subsystem. Used for simulation updates.
   *
   * @return the drive subsystem
   */
  public Drive getDrive() {
    return drive;
  }

  /**
   * Gets the fuel sim. Null when not in SIM.
   *
   * @return the FuelSim instance or null
   */
  public FuelSim getFuelSim() {
    return fuelSim;
  }
}
