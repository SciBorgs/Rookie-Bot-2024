package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import monologue.Logged;

/** Hardware interface for a differential drivetrain. */
public interface DifferentialDriveIO extends AutoCloseable, Subsystem, Logged {
  /** Sets the voltage of the left side. */
  public Command setLeftVoltage(Measure<Voltage> voltage);

  /**
   * Sets the voltage of the left side.
   *
   * @param volts : Voltage(volts).
   */
  default void setLeftVoltage(double volts) {
    CommandScheduler.getInstance().schedule(setLeftVoltage(Volts.of(volts)));
  }

  /** Returns the net displacement that the left side has traveled. */
  public Measure<Distance> getLeftDisplacement();

  /**
   * Returns the net displacement that the left side has traveled.
   *
   * @return Displacement(Meters).
   */
  default double getLeftDisplacementDouble() {
    return getLeftDisplacement().in(Meters);
  }

  /** Returns the current velocity of the left side. */
  public Measure<Velocity<Distance>> getLeftVelocity();

  /**
   * Returns the current velocity of the left side.
   *
   * @return Velocity(Meters per second).
   */
  default double getLeftVelocityDouble() {
    return getLeftVelocity().in(MetersPerSecond);
  }

  /** Resets displacement measurement of the left side. */
  public void resetLeftEncoder();

  /** Sets the voltage of the right side. */
  public Command setRightVoltage(Measure<Voltage> voltage);

  /**
   * Sets the voltage of the right side.
   *
   * @param volts : Voltage(volts).
   */
  default void setRightVoltage(double volts) {
    CommandScheduler.getInstance().schedule(setRightVoltage(Volts.of(volts)));
  }

  /** Returns the net displacement that the right side has traveled. */
  public Measure<Distance> getRightDisplacement();

  /**
   * Returns the net displacement that the right side has traveled.
   *
   * @return Displacement(Meters).
   */
  default double getRightDisplacementDouble() {
    return getRightDisplacement().in(Meters);
  }

  /** Returns the current velocity of the right side. */
  public Measure<Velocity<Distance>> getRightVelocity();

  /**
   * Returns the current velocity of the right side.
   *
   * @return Velocity(Meters per second).
   */
  default double getRightVelocityDouble() {
    return getRightVelocity().in(MetersPerSecond);
  }

  /** Resets displacement measurement of the right side. */
  public void resetRightEncoder();

  /** Resets displacement measurement of both sides. */
  public void resetEncoders();

  /** Returns estimated pose of the drivetrain(In Meters). */
  public Pose2d getPose();

  /**
   * Updates estimated pose of the drivetrain.
   *
   * @param deltaTime Time since last odometry update.
   */
  public void updatePose(Measure<Time> deltaTime);

  public class NoDiffDrive implements DifferentialDriveIO {
    @Override
    public void close() throws Exception {}

    @Override
    public Command setLeftVoltage(Measure<Voltage> voltage) {
      return Commands.idle(this);
    }

    @Override
    public Measure<Distance> getLeftDisplacement() {
      return Meters.of(0);
    }

    @Override
    public Measure<Velocity<Distance>> getLeftVelocity() {
      return MetersPerSecond.of(0);
    }

    @Override
    public void resetLeftEncoder() {}

    @Override
    public Command setRightVoltage(Measure<Voltage> voltage) {
      return Commands.idle(this);
    }

    @Override
    public Measure<Distance> getRightDisplacement() {
      return Meters.of(0);
    }

    @Override
    public Measure<Velocity<Distance>> getRightVelocity() {
      return MetersPerSecond.of(0);
    }

    @Override
    public void resetRightEncoder() {}

    @Override
    public void resetEncoders() {}

    @Override
    public Pose2d getPose() {
      return new Pose2d();
    }

    @Override
    public void updatePose(Measure<Time> deltaTime) {}

    /** Creates a new instance of this DifferentialDriveIO class. */
    public DifferentialDriveIO create() {
      return new NoDiffDrive();
    }
  }
}
