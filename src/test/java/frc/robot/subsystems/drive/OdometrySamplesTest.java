package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class OdometrySamplesTest {
  @Test
  void acceptsOnlyCompleteCommonTimestampSamples() {
    double[][] times = {{1, 1.004}, {1, 1.004}, {1, 1.004}, {1, 1.004}};
    int[] counts = {2, 2, 2, 2};
    assertTrue(OdometrySamples.aligned(1, 1.004, times, counts, true, times[0], 2));
    counts[2] = 1;
    assertFalse(OdometrySamples.aligned(1, 1.004, times, counts, true, times[0], 2));
    counts[2] = 2;
    times[2][1] = 1.008;
    assertFalse(OdometrySamples.aligned(1, 1.004, times, counts, true, times[0], 2));
  }

  @Test
  void missingGyroSamplesAreRejectedButDisconnectedGyroAllowsWheelFallback() {
    double[][] times = {{1}, {1}, {1}, {1}};
    int[] counts = {1, 1, 1, 1};
    assertFalse(OdometrySamples.aligned(0, 1, times, counts, true, new double[0], 0));
    assertTrue(OdometrySamples.aligned(0, 1, times, counts, false, new double[0], 0));
    assertFalse(OdometrySamples.aligned(0, Double.NaN, times, counts, false, new double[0], 0));
  }
}
