// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.simulation.FuelSim;
import frc.robot.simulation.SimulatedRobotState;
import frc.robot.subsystems.intake.IntakeConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  private Orchestra orchestra_all;

  private Orchestra voice1;
  private Orchestra voice2;
  private Orchestra voice3;
  private Orchestra voice4;

  TalonFX[] motors = {
    new TalonFX(1),  // FL Drive
    new TalonFX(2),  // FL Steer
    new TalonFX(3),  // BL Drive
    new TalonFX(4),  // BL Steer
    new TalonFX(5),  // BR Drive
    new TalonFX(6),  // BR Steer
    new TalonFX(7),  // FR Drive
    new TalonFX(8),  // FR Steer
    new TalonFX(22), // Hood
    new TalonFX(23), // Intake Wrist
    new TalonFX(25), // Intake Wheel
    new TalonFX(30), // Feeder
    new TalonFX(31), // Turret
    new TalonFX(35), // Serializer
    new TalonFX(38), // Shooter Follower
    new TalonFX(40), // Shooter
  };

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    // Instantiate Orchestra

    for (int i = 0; i < motors.length; ++i) {
        orchestra_all.addInstrument(motors[i]);
    }
    
    voice1.addInstrument(new TalonFX(1)); // FL drive motor
    voice2.addInstrument(new TalonFX(7)); // FR drive motor
    voice3.addInstrument(new TalonFX(3)); // BL drive motor
    voice4.addInstrument(new TalonFX(5)); // BR drive motor

    // orchestra_all.loadMusic("blah");
    orchestra_all.loadMusic("bwv_846.mid");
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    RobotState state = RobotState.getInstance();
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);

    // Show in sim and replay simulation
    if (Constants.currentMode != Constants.Mode.REAL) {
      robotContainer.updateShooterTrajectoryVisualization();
    }

    Rotation3d turretYaw =
        new Rotation3d(0, 0, state.getLatestTurretAngle().getValue().in(Radians));

    Rotation3d hoodPitch =
        new Rotation3d(
            0, (Math.PI / 2) - state.getLatestHoodPosition().getValue().in(Radians) - 0.3054325, 0);

    Pose3d turretPose =
        Constants.turretBasePose.transformBy(new Transform3d(new Translation3d(), turretYaw));

    Pose3d hoodPose =
        turretPose
            .transformBy(Constants.turretToHood)
            .transformBy(new Transform3d(new Translation3d(), hoodPitch));

    double intakePercentage =
        state.getLatestIntakeWristPosition().getValue().in(Radians)
            / IntakeConstants.kMaxWristAngle.in(Radians);
    Pose3d hopperPose =
        Constants.hopperBasePose.transformBy(
            new Transform3d(
                new Translation3d(
                    -(intakePercentage * Constants.kHopperMaxExtension.in(Meters)), 0, 0),
                new Rotation3d()));

    Pose3d intakePose =
        Constants.intakeBasePose.transformBy(
            new Transform3d(
                new Translation3d(),
                new Rotation3d(
                    0, -state.getLatestIntakeWristPosition().getValue().in(Radians), 0)));
    Logger.recordOutput(
        "ComponentPoses", new Pose3d[] {turretPose, hoodPose, intakePose, hopperPose});
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    orchestra.play();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      SimulatedRobotState simulatedRobotState = SimulatedRobotState.getInstance();
      var drive = robotContainer.getDrive();
      if (drive != null) {
        var groundTruthPose = drive.getOdometryOnlyPose();
        simulatedRobotState.addFieldToRobot(groundTruthPose);
      }
      var fuelSim = robotContainer.getFuelSim();
      if (fuelSim != null) {
        fuelSim.updateSim();
        Logger.recordOutput("FuelSim/BlueHubScore", FuelSim.Hub.BLUE_HUB.getScore());
        Logger.recordOutput("FuelSim/RedHubScore", FuelSim.Hub.RED_HUB.getScore());
      }
    }
  }
}
