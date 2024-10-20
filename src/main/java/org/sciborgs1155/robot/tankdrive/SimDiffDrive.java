package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.MOI_MASS;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.REDUCTION;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.ROBOT_MASS;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.STARTING_POSE;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.STD_DEVS;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.TRACK_WIDTH;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.WHEEL_RADIUS;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Simulates a differential drivetrain with 4 REV NEO Motors. */
public class SimDiffDrive implements DifferentialDriveIO {
  /** Simulation of 4 REV NEO motors. */
  private final DifferentialDrivetrainSim simulation =
      new DifferentialDrivetrainSim(
          DCMotor.getNEO(2),
          REDUCTION,
          MOI_MASS.in(Kilograms),
          ROBOT_MASS.in(Kilograms),
          WHEEL_RADIUS.in(Meters),
          TRACK_WIDTH.in(Meters),
          STD_DEVS);

  /** Wheel voltages(volts). */
  private final DifferentialDriveWheelVoltages voltages = new DifferentialDriveWheelVoltages(0, 0);

  /** If this class is closed, the simulation will stop updating. */
  private boolean isClosed = false;

  @Override
  public Command setLeftVoltage(Measure<Voltage> voltage) {
    return runOnce(() -> voltages.left = voltage.in(Volts))
        .withName("setLeftVoltage(" + voltage.in(Volts) + ")")
        .andThen(Commands.idle(this));
  }

  @Override
  public Measure<Distance> getLeftDisplacement() {
    return Meters.of(simulation.getLeftPositionMeters());
  }

  @Override
  public Measure<Velocity<Distance>> getLeftVelocity() {
    return MetersPerSecond.of(simulation.getLeftVelocityMetersPerSecond());
  }

  /** NOTE: Does nothing. */
  @Override
  public void resetLeftEncoder() {}

  @Override
  public Command setRightVoltage(Measure<Voltage> voltage) {
    return runOnce(() -> voltages.right = voltage.in(Volts))
        .withName("setRightVoltage(" + voltage.in(Volts) + ")")
        .andThen(Commands.idle(this));
  }

  @Override
  public Measure<Distance> getRightDisplacement() {
    return Meters.of(simulation.getRightPositionMeters());
  }

  @Override
  public Measure<Velocity<Distance>> getRightVelocity() {
    return MetersPerSecond.of(simulation.getRightVelocityMetersPerSecond());
  }

  /** NOTE: Does nothing. */
  @Override
  public void resetRightEncoder() {}

  /** NOTE: Does nothing. */
  @Override
  public void resetEncoders() {
    resetLeftEncoder();
    resetRightEncoder();
  }

  @Override
  public void close() throws Exception {}

  /** Creates a new instance of this DifferentialDriveIO class. */
  public static SimDiffDrive create() {
    return new SimDiffDrive();
  }

  /** Creates a new instance of this DifferentialDriveIO class. */
  private SimDiffDrive() {
    // Sets the starting pose of the simulation.
    simulation.setPose(STARTING_POSE);
  }

  @Override
  public Pose2d getPose() {
    return simulation.getPose();
  }

  @Override
  public void updatePose(Measure<Time> deltaTime) {
    if (isClosed) {
      return;
    }
    simulation.setInputs(voltages.left, voltages.right);
    simulation.update(deltaTime.in(Seconds));
  }
}
