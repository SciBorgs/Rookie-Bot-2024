package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.MOMENT_OF_INERTIA;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.REDUCTION;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.ROBOT_MASS;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.STANDARD_MEASUREMENT_DEVIATIONS;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.STARTING_POSE;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.TRACK_WIDTH;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.WHEEL_RADIUS;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.clampVoltage;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

/** Simulated {@link DiffDriveIO} class using 4 NEOs. */
public class SimDiffDrive implements DiffDriveIO {
  /** Simulated drivetrain with 4 NEO motors */
  private final DifferentialDrivetrainSim simulation =
      new DifferentialDrivetrainSim(
          DCMotor.getNEO(2),
          REDUCTION,
          MOMENT_OF_INERTIA.in(Kilograms),
          ROBOT_MASS.in(Kilograms),
          WHEEL_RADIUS.in(Meters),
          TRACK_WIDTH.in(Meters),
          STANDARD_MEASUREMENT_DEVIATIONS);

  /**
   * Current Left and Right voltages of the drivetrain (since both have to be updated at once with
   * sim, they are updated in the 'updatePose'method). These values are updated using the
   * 'set[Left/Right]Voltage' methods
   */
  private final DifferentialDriveWheelVoltages voltages = new DifferentialDriveWheelVoltages(0, 0);

  /** Around robot orgin(for use with 'getAngularVelocity' method) */
  private Measure<Velocity<Angle>> angularVelocity = DegreesPerSecond.of(0);

  /** For calculating angular velocity(rotation at previous simulation update) */
  private Measure<Angle> previousRotation = Degrees.of(0);

  @Override
  public Measure<Voltage> setLeftVoltage(Measure<Voltage> voltage) {
    voltages.left = clampVoltage(voltage).in(Volts);
    return voltage;
  }

  @Override
  public Measure<Distance> getLeftDisplacement() {
    return Meters.of(simulation.getLeftPositionMeters());
  }

  @Override
  public Measure<Velocity<Distance>> getLeftVelocity() {
    return MetersPerSecond.of(simulation.getLeftVelocityMetersPerSecond());
  }

  /** NOTE: you can't reset sim encoders (This method does absolutely nothing) */
  @Override
  public void resetLeftEncoder() {}

  @Override
  public Measure<Voltage> setRightVoltage(Measure<Voltage> voltage) {
    voltages.right = clampVoltage(voltage).in(Volts);
    return voltage;
  }

  @Override
  public Measure<Distance> getRightDisplacement() {
    return Meters.of(simulation.getRightPositionMeters());
  }

  @Override
  public Measure<Velocity<Distance>> getRightVelocity() {
    return MetersPerSecond.of(simulation.getRightVelocityMetersPerSecond());
  }

  @Override
  public Measure<Velocity<Angle>> getAngularVelocity() {
    return angularVelocity;
  }

  /** NOTE: you can't reset sim encoders (This method does absolutely nothing) */
  @Override
  public void resetRightEncoder() {}

  public SimDiffDrive() {
    simulation.setPose(STARTING_POSE);
  }

  @Override
  public Pose2d getPose() {
    return simulation.getPose();
  }

  /** NOTE: you can't reset sim encoders (This method does absolutely nothing) */
  @Override
  public void resetEncoders() {}

  @Override
  public Pose2d updatePose(Measure<Time> deltaTime) {
    simulation.setInputs(voltages.left, voltages.right);
    simulation.update(deltaTime.in(Seconds));

    Measure<Angle> currentRotation = Degrees.of(simulation.getPose().getRotation().getDegrees());
    Measure<Angle> deltaRotation = currentRotation.minus(previousRotation);

    angularVelocity = DegreesPerSecond.of(deltaRotation.in(Degrees) / deltaTime.in(Seconds));
    previousRotation = currentRotation;

    return getPose();
  }
}
