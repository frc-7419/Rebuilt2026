package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class ShooterOutputTest {
  @Test
  void newTargetsAreSentImmediatelyAndPeriodicHandoffRetainsFollowerDirection() {
    double[] actual = {0};
    List<ControlRequest> lead = new ArrayList<>();
    List<ControlRequest> follow = new ArrayList<>();
    var c = new ShooterIOTalonFX.ClosedLoop(lead::add, follow::add, () -> actual[0]);
    c.setWheelRps(80);
    assertEquals(1, lead.size()); // No periodic update needed to send the new goal.
    assertEquals(ShooterConstants.kShooterBangStatorAmps, ((TorqueCurrentFOC) lead.get(0)).Output);
    assertEquals(
        -ShooterConstants.kShooterBangStatorAmps, ((TorqueCurrentFOC) follow.get(0)).Output);
    actual[0] = 80 * ShooterConstants.kMotorToShooterGearRatio;
    c.apply();
    assertEquals(actual[0], ((VelocityVoltage) lead.get(1)).Velocity, 1e-10);
    assertEquals(-actual[0], ((VelocityVoltage) follow.get(1)).Velocity, 1e-10);
    actual[0] = 0;
    c.apply();
    assertInstanceOf(TorqueCurrentFOC.class, lead.get(2));
    c.setWheelRps(0);
    assertEquals(4, lead.size());
    assertEquals(0, ((VelocityVoltage) lead.get(3)).Velocity);
    assertFalse(((VelocityVoltage) lead.get(3)).EnableFOC);
  }

  @Test
  void reverseSetpointUsesMatchingNegativeBangAndPositiveFollowerCurrent() {
    List<ControlRequest> lead = new ArrayList<>();
    List<ControlRequest> follow = new ArrayList<>();
    var c = new ShooterIOTalonFX.ClosedLoop(lead::add, follow::add, () -> 0);
    c.setWheelRps(-80);
    assertEquals(-ShooterConstants.kShooterBangStatorAmps, ((TorqueCurrentFOC) lead.get(0)).Output);
    assertEquals(
        ShooterConstants.kShooterBangStatorAmps, ((TorqueCurrentFOC) follow.get(0)).Output);
  }
}
