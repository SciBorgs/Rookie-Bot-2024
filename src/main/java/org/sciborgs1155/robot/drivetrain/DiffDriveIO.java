package org.sciborgs1155.robot.drivetrain;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

/** Hardware interface for a differential drivetrain */
public interface DiffDriveIO {
  /**
   * Sets the voltage of the left side of the drivetrain
   *
   * @return The new left voltage
   */
  public Measure<Voltage> setLeftVoltage(Measure<Voltage> voltage);

  /** Total displacement of the left side of the drivetrain */
  public Measure<Distance> getLeftDisplacement();

  /** The current velocity of the left side of the drivetrain */
  public Measure<Velocity<Distance>> getLeftVelocity();

  /** Resets displacement measurement of the left side of the drivetrain */
  public void resetLeftEncoder();

  /**
   * Sets the voltage of the right side
   *
   * @return The new right voltage
   */
  public Measure<Voltage> setRightVoltage(Measure<Voltage> voltage);

  /** Total displacement of the right side of the drivetrain */
  public Measure<Distance> getRightDisplacement();

  /** The current velocity of the right side of the drivetrain */
  public Measure<Velocity<Distance>> getRightVelocity();

  /** Resets displacement measurement of the right side of the drivetrain */
  public void resetRightEncoder();

  /** Resets displacement measurement of both sides of the drivetrain */
  default void resetEncoders() {
    resetLeftEncoder();
    resetRightEncoder();
  }
  ;

  /** The current position of the drivetrain(Translation is in Meters) */
  public Pose2d getPose();

  /** The angular velocity of the drivetrain(counterclockwise) */
  public Measure<Velocity<Angle>> getAngularVelocity();

  /**
   * Updates odometry measurements
   *
   * @param deltaTime : Time since last odometry update
   * @return The current position of the drivetrain
   */
  public Pose2d updatePose(Measure<Time> deltaTime);

  /** Disfunctional Placeholder {@link DiffDriveIO} class */
  public class NoDiffDrive implements DiffDriveIO {

    @Override
    public Measure<Voltage> setLeftVoltage(Measure<Voltage> voltage) {
      return Volts.of(0);
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
    public Measure<Voltage> setRightVoltage(Measure<Voltage> voltage) {
      return Volts.of(0);
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
    public Pose2d updatePose(Measure<Time> deltaTime) {
      return new Pose2d();
    }

    @Override
    public Measure<Velocity<Angle>> getAngularVelocity() {
      return DegreesPerSecond.of(0);
    }
  }
}
