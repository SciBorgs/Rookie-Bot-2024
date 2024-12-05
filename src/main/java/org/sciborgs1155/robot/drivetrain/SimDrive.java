package org.sciborgs1155.robot.drivetrain;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.drivetrain.DriveConstants.GEARING;
import static org.sciborgs1155.robot.drivetrain.DriveConstants.MOI;
import static org.sciborgs1155.robot.drivetrain.DriveConstants.ROBOT_MASS;
import static org.sciborgs1155.robot.drivetrain.DriveConstants.STARTING_POSE;
import static org.sciborgs1155.robot.drivetrain.DriveConstants.TRACK_WIDTH;
import static org.sciborgs1155.robot.drivetrain.DriveConstants.WHEEL_RADIUS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import monologue.Annotations.Log;

/** Simulated {@link DriveIO} class using 4 NEOs */
public class SimDrive implements DriveIO {
  /** Simulated drivetrain with 4 NEO motors */
  private final DifferentialDrivetrainSim simulation =
      new DifferentialDrivetrainSim(
          DCMotor.getNEO(2),
          GEARING,
          MOI,
          ROBOT_MASS.in(Kilograms),
          WHEEL_RADIUS.in(Meters),
          TRACK_WIDTH.in(Meters),
          VecBuilder.fill(0.00, 0.00, 0.00, 0., 0., 0.00, 0.00)); // Assume sim is 100% accurate

  /**
   * Current Left and Right voltages of the drivetrain (since both have to be updated at once in the
   * 'updatePose' method). These values are updated using the set[Left/Right]Voltage' methods
   */
  private final DifferentialDriveWheelVoltages voltages = new DifferentialDriveWheelVoltages(0, 0);

  /** Last timestamp in whih the simulation was updated(seconds) */
  private double lastTime = 0;

  public SimDrive() {
    simulation.setPose(STARTING_POSE);
  }

  @Override
  public void setLeftVoltage(double volts) {
    voltages.left = clampVoltage(volts);
  }

  @Override
  @Log.NT
  public double getLeftDisplacement() {
    return simulation.getLeftPositionMeters();
  }

  @Override
  @Log.NT
  public double getLeftVelocity() {
    return simulation.getLeftVelocityMetersPerSecond();
  }

  /** NOTE: you can't reset sim encoders (This method does absolutely nothing) */
  @Override
  public void resetLeftEncoder() {}

  @Override
  public void setRightVoltage(double volts) {
    voltages.right = clampVoltage(volts);
  }

  @Override
  @Log.NT
  public double getRightDisplacement() {
    return (simulation.getRightPositionMeters());
  }

  @Override
  @Log.NT
  public double getRightVelocity() {
    return simulation.getRightVelocityMetersPerSecond();
  }

  /** NOTE: you can't reset sim encoders (This method does absolutely nothing) */
  @Override
  public void resetRightEncoder() {}

  /** NOTE: you can't reset sim encoders (This method does absolutely nothing) */
  @Override
  public void resetEncoders() {}

  @Override
  public void update() {
    simulation.setInputs(voltages.left, voltages.right);
    simulation.update(Timer.getFPGATimestamp() - lastTime);
    lastTime = Timer.getFPGATimestamp();
  }
}
