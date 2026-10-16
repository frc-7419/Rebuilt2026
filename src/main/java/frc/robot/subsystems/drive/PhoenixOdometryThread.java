// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.generated.TunerConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class PhoenixOdometryThread extends Thread {
  private final Lock signalsLock =
      new ReentrantLock(); // Prevents conflicts when registering signals
  private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
  private final List<Boolean> requiredSignals = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private static boolean isCANFD = TunerConstants.kCANBus.isNetworkFD();
  private static PhoenixOdometryThread instance = null;
  private volatile long overflowCount;
  private volatile long errorCount;
  private volatile String lastStatus = "NotStarted";

  public long getOverflowCount() {
    return overflowCount;
  }

  public long getErrorCount() {
    return errorCount;
  }

  public String getLastStatus() {
    return lastStatus;
  }

  private boolean queuesHaveCapacity() {
    for (Queue<Double> queue : phoenixQueues) if (queue.size() >= 20) return false;
    for (Queue<Double> queue : genericQueues) if (queue.size() >= 20) return false;
    for (Queue<Double> queue : timestampQueues) if (queue.size() >= 20) return false;
    return true;
  }

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  private PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

  @Override
  public void start() {
    if (timestampQueues.size() > 0) {
      super.start();
    }
  }

  /** Registers a Phoenix signal to be read from the thread. */
  public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
    return registerSignal(signal, true);
  }

  /** The gyro is optional so wheel odometry continues when it disconnects. */
  public Queue<Double> registerSignal(StatusSignal<Angle> signal, boolean required) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    Drive.odometryLock.lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
      System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
      newSignals[phoenixSignals.length] = signal;
      phoenixSignals = newSignals;
      phoenixQueues.add(queue);
      requiredSignals.add(required);
    } finally {
      signalsLock.unlock();
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Registers a generic signal to be read from the thread. */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    Drive.odometryLock.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      signalsLock.unlock();
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Returns a new queue that returns timestamp values for each sample. */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      // Wait for updates from all signals
      StatusCode status = StatusCode.OK;
      signalsLock.lock();
      try {
        if (isCANFD && phoenixSignals.length > 0) {
          status = BaseStatusSignal.waitForAll(2.0 / Drive.ODOMETRY_FREQUENCY, phoenixSignals);
        } else {
          // "waitForAll" does not support blocking on multiple signals with a bus
          // that is not CAN FD, regardless of Pro licensing. No reasoning for this
          // behavior is provided by the documentation.
          Thread.sleep((long) (1000.0 / Drive.ODOMETRY_FREQUENCY));
          if (phoenixSignals.length > 0) status = BaseStatusSignal.refreshAll(phoenixSignals);
        }
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        return;
      } finally {
        signalsLock.unlock();
      }

      lastStatus = status.toString();
      if (!status.isOK()) errorCount++;

      // Save new data to queues
      Drive.odometryLock.lock();
      try {
        // Drop the entire frame if any queue is full; partial offers break sample alignment.
        if (!queuesHaveCapacity()) {
          overflowCount++;
          continue;
        }
        // Sample timestamp is current FPGA time minus average CAN latency
        // Default timestamps from Phoenix are NOT compatible with
        // FPGA timestamps, this solution is imperfect but close
        double timestamp = RobotController.getFPGATime() / 1e6;
        double totalLatency = 0.0;
        int validSignals = 0;
        boolean requiredSignalsValid = true;
        for (int i = 0; i < phoenixSignals.length; i++) {
          var signal = phoenixSignals[i];
          double latency = signal.getTimestamp().getLatency();
          boolean usable =
              signal.getStatus().isOK()
                  && Double.isFinite(latency)
                  && latency >= 0.0
                  && latency <= 0.1;
          if (requiredSignals.get(i) && !usable) requiredSignalsValid = false;
          if (usable) {
            totalLatency += latency;
            validSignals++;
          }
        }
        if (!requiredSignalsValid || (phoenixSignals.length > 0 && validSignals == 0)) continue;
        if (validSignals > 0) timestamp -= totalLatency / validSignals;

        // Add new samples to queues
        for (int i = 0; i < phoenixSignals.length; i++) {
          phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
        }
        for (int i = 0; i < genericSignals.size(); i++) {
          genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
        }
        for (int i = 0; i < timestampQueues.size(); i++) {
          timestampQueues.get(i).offer(timestamp);
        }
      } finally {
        Drive.odometryLock.unlock();
      }
    }
  }
}
