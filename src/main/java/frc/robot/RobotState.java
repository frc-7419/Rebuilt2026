package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  private static final double LOOKBACK_TIME_SEC = 2.0;
  private static final Matrix<N3, N1> odometryStateStdDevs =
      new Matrix<>(
          VecBuilder.fill(
              Meters.of(0.003).in(Meters),
              Meters.of(0.003).in(Meters),
              Radians.of(0.002).in(Radians)));

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  private final TimeInterpolatableBuffer<Pose2d> fieldToRobot =
      TimeInterpolatableBuffer.createBuffer(LOOKBACK_TIME_SEC);
  private final TimeInterpolatableBuffer<Double> turretAngle =
      TimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME_SEC);
  private final TimeInterpolatableBuffer<Double> turretAngularVelocity =
      TimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME_SEC);
  private final TimeInterpolatableBuffer<Double> hoodPosition =
      TimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME_SEC);
  private final TimeInterpolatableBuffer<Double> shooterVelocity =
      TimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME_SEC);
  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
  private final TimeInterpolatableBuffer<Double> shooterRotorVelocity =
      TimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME_SEC);

  // Intake buffers
  private final TimeInterpolatableBuffer<Double> intakeWristPosition =
      TimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME_SEC);
  private final TimeInterpolatableBuffer<Double> intakeWristVelocity =
      TimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME_SEC);

  // Hopper buffers
  private final TimeInterpolatableBuffer<Double> hopperVelocity =
      TimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME_SEC);

  private final AtomicReference<ChassisSpeeds> measuredRobotRelativeChassisSpeeds =
      new AtomicReference<>(new ChassisSpeeds());
  private final AtomicReference<ChassisSpeeds> measuredFieldRelativeChassisSpeeds =
      new AtomicReference<>(new ChassisSpeeds());
  private final AtomicReference<ChassisSpeeds> desiredFieldRelativeChassisSpeeds =
      new AtomicReference<>(new ChassisSpeeds());

  @AutoLogOutput private Pose2d estimatedPose = Pose2d.kZero;

  private RobotState() {
    for (int i = 0; i < 3; ++i) {
      qStdDevs.set(i, 0, Math.pow(odometryStateStdDevs.get(i, 0), 2));
    }
    fieldToRobot.addSample(0.0, Pose2d.kZero);
    turretAngle.addSample(0.0, 0.0);
    turretAngularVelocity.addSample(0.0, 0.0);
    hoodPosition.addSample(0.0, 0.0);
    shooterVelocity.addSample(0.0, 0.0);
    shooterRotorVelocity.addSample(0.0, 0.0);
    intakeWristPosition.addSample(0.0, 0.0);
    intakeWristVelocity.addSample(0.0, 0.0);
    hopperVelocity.addSample(0.0, 0.0);
  }

  public void resetPose(Pose2d pose) {
    estimatedPose = pose;
    fieldToRobot.clear();
    fieldToRobot.addSample(0.0, pose);
  }

  public void addOdometryMeasurement(double timestamp, Pose2d pose) {
    fieldToRobot.addSample(timestamp, pose);
    estimatedPose = pose;
  }

  public void addDriveMotionMeasurements(
      double timestamp,
      ChassisSpeeds desiredFieldRelativeSpeeds,
      ChassisSpeeds measuredSpeeds,
      ChassisSpeeds measuredFieldRelativeSpeeds) {
    this.desiredFieldRelativeChassisSpeeds.set(desiredFieldRelativeSpeeds);
    this.measuredRobotRelativeChassisSpeeds.set(measuredSpeeds);
    this.measuredFieldRelativeChassisSpeeds.set(measuredFieldRelativeSpeeds);
  }

  public Optional<Pose2d> getFieldToRobot(double timestamp) {
    return fieldToRobot.getSample(timestamp);
  }

  public Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
    var buffer = fieldToRobot.getInternalBuffer();
    return buffer.isEmpty() ? null : buffer.lastEntry();
  }

  public Pose2d getLatestFieldToRobotPose() {
    var latest = getLatestFieldToRobot();
    return latest != null ? latest.getValue() : Pose2d.kZero;
  }

  public Pose2d getPredictedFieldToRobot(double lookaheadTimeS) {
    var maybeFieldToRobot = getLatestFieldToRobot();
    Pose2d fieldToRobot = maybeFieldToRobot == null ? Pose2d.kZero : maybeFieldToRobot.getValue();
    var speeds = getLatestRobotRelativeChassisSpeed();
    return fieldToRobot.exp(
        new Twist2d(
            speeds.vxMetersPerSecond * lookaheadTimeS,
            speeds.vyMetersPerSecond * lookaheadTimeS,
            speeds.omegaRadiansPerSecond * lookaheadTimeS));
  }

  public Rotation2d getRotation() {
    return estimatedPose.getRotation();
  }

  @AutoLogOutput
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        measuredRobotRelativeChassisSpeeds.get(), getRotation());
  }

  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  public ChassisSpeeds getLatestRobotRelativeChassisSpeed() {
    return measuredRobotRelativeChassisSpeeds.get();
  }

  public ChassisSpeeds getLatestMeasuredFieldRelativeChassisSpeeds() {
    return measuredFieldRelativeChassisSpeeds.get();
  }

  public ChassisSpeeds getLatestDesiredFieldRelativeChassisSpeed() {
    return desiredFieldRelativeChassisSpeeds.get();
  }

  public TimeInterpolatableBuffer<Pose2d> getPoseBuffer() {
    return fieldToRobot;
  }

  public double getPoseBufferSizeSec() {
    return LOOKBACK_TIME_SEC;
  }

  public Matrix<N3, N1> getQStdDevs() {
    return qStdDevs;
  }

  public void setEstimatedPose(Pose2d pose) {
    estimatedPose = pose;
  }

  public void addTurretUpdates(
      double timestamp, Angle turretRotation, AngularVelocity turretAngularVelocityMeasure) {
    turretAngle.addSample(timestamp, turretRotation.in(Radians));
    turretAngularVelocity.addSample(timestamp, turretAngularVelocityMeasure.baseUnitMagnitude());
  }

  public void addShooterUpdates(
      double timestamp, AngularVelocity shooterWheelVelocity, AngularVelocity rotorVelocity) {
    // Store in base units (rad/s) so it’s consistent regardless of the caller using RPM, RPS, etc.
    shooterVelocity.addSample(timestamp, shooterWheelVelocity.baseUnitMagnitude());
    shooterRotorVelocity.addSample(timestamp, rotorVelocity.baseUnitMagnitude());
  }

  public Optional<Angle> getTurretAngle(double timestamp) {
    return turretAngle.getSample(timestamp).map(Radians::of);
  }

  public Map.Entry<Double, Angle> getLatestTurretAngle() {
    var buffer = turretAngle.getInternalBuffer();
    if (buffer.isEmpty()) return null;
    var last = buffer.lastEntry();
    return Map.entry(last.getKey(), Radians.of(last.getValue()));
  }

  public AngularVelocity getLatestTurretAngularVelocity() {
    var buffer = turretAngularVelocity.getInternalBuffer();
    double value = buffer.isEmpty() ? 0.0 : buffer.lastEntry().getValue();
    return RadiansPerSecond.of(value);
  }

  public void addHoodUpdates(double timestamp, Angle hoodPositionMeasure) {
    hoodPosition.addSample(timestamp, hoodPositionMeasure.baseUnitMagnitude());
  }

  public Map.Entry<Double, Angle> getLatestHoodPosition() {
    var buffer = hoodPosition.getInternalBuffer();
    if (buffer.isEmpty()) return null;
    var last = buffer.lastEntry();
    return Map.entry(last.getKey(), Radians.of(last.getValue()));
  }

  public AngularVelocity getLatestShooterVelocity() {
    var buffer = shooterVelocity.getInternalBuffer();
    double value = buffer.isEmpty() ? 0.0 : buffer.lastEntry().getValue();
    // return RadiansPerSecond.of(value);
    return RotationsPerSecond.of(5);
  }

  public boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }

  public Optional<Character> getGameData() {
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.length() > 0) {
      return Optional.of(gameData.charAt(0));
    }
    return Optional.empty();
  }

  public enum MatchShift {
    AUTO,
    TELEOP_TRANSITION,
    SHIFT_1,
    SHIFT_2,
    SHIFT_3,
    SHIFT_4,
    END_GAME,
    UNKNOWN
  }

  public MatchShift getCurrentShift() {
    if (!DriverStation.isEnabled()) {
      return MatchShift.UNKNOWN;
    }

    double matchTime = DriverStation.getMatchTime();
    if (matchTime > 130.0) {
      return MatchShift.AUTO;
    } else if (matchTime > 105.0) {
      return MatchShift.TELEOP_TRANSITION;
    } else if (matchTime > 80.0) {
      return MatchShift.SHIFT_1;
    } else if (matchTime > 55.0) {
      return MatchShift.SHIFT_2;
    } else if (matchTime > 30.0) {
      return MatchShift.SHIFT_3;
    } else if (matchTime > 0.0) {
      return MatchShift.SHIFT_4;
    } else {
      return MatchShift.END_GAME;
    }
  }

  public boolean isHubActive() {
    var gameData = getGameData();
    if (gameData.isEmpty()) {
      return false;
    }

    char inactiveAlliance = gameData.get();
    MatchShift currentShift = getCurrentShift();

    if (currentShift != MatchShift.SHIFT_2 && currentShift != MatchShift.SHIFT_4) {
      return false;
    }

    boolean isRed = isRedAlliance();
    boolean inactiveIsRed = inactiveAlliance == 'R';

    return isRed == inactiveIsRed;
  }

  public void addIntakeUpdates(
      double timestamp, Angle wristPosition, AngularVelocity wristVelocity) {
    intakeWristPosition.addSample(timestamp, wristPosition.baseUnitMagnitude());
    intakeWristVelocity.addSample(timestamp, wristVelocity.baseUnitMagnitude());
  }

  public Optional<Angle> getIntakeWristPosition(double timestamp) {
    return intakeWristPosition.getSample(timestamp).map(Radians::of);
  }

  public Map.Entry<Double, Angle> getLatestIntakeWristPosition() {
    var buffer = intakeWristPosition.getInternalBuffer();
    if (buffer.isEmpty()) return null;
    var last = buffer.lastEntry();
    return Map.entry(last.getKey(), Radians.of(last.getValue()));
  }

  public AngularVelocity getLatestIntakeWristVelocity() {
    var buffer = intakeWristVelocity.getInternalBuffer();
    double value = buffer.isEmpty() ? 0.0 : buffer.lastEntry().getValue();
    return RadiansPerSecond.of(value);
  }

  public void addHopperUpdates(double timestamp, AngularVelocity velocity) {
    hopperVelocity.addSample(timestamp, velocity.baseUnitMagnitude());
  }

  public Optional<AngularVelocity> getHopperVelocity(double timestamp) {
    return hopperVelocity.getSample(timestamp).map(RadiansPerSecond::of);
  }

  public AngularVelocity getLatestHopperVelocity() {
    var buffer = hopperVelocity.getInternalBuffer();
    double value = buffer.isEmpty() ? 0.0 : buffer.lastEntry().getValue();
    return RadiansPerSecond.of(value);
  }

  public record VisionObservation(double timestamp, Pose2d visionPose, Matrix<N3, N1> stdDevs) {}
}
