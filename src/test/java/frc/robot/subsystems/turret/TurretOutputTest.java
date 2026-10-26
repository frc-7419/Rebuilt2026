package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class TurretOutputTest {
  @Test
  void positionAndVelocityAreSentImmediatelyInMechanismRotationsWithOffset() {
    List<double[]> sent = new ArrayList<>();
    var control =
        new TurretIOTalonFX.PositionControl(
            request -> sent.add(new double[] {request.Position, request.Velocity}));
    control.setState(Degrees.of(180), RadiansPerSecond.of(Math.PI), Degrees.of(30));
    assertEquals(1, sent.size());
    assertEquals(150.0 / 360.0, sent.get(0)[0], 1e-10);
    assertEquals(.5, sent.get(0)[1], 1e-10);
    control.setState(Degrees.of(-90), RadiansPerSecond.of(-2 * Math.PI), Degrees.of(30));
    assertEquals(2, sent.size());
    assertEquals(-120.0 / 360.0, sent.get(1)[0], 1e-10);
    assertEquals(-1, sent.get(1)[1], 1e-10);
  }
}
