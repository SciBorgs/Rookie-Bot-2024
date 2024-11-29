package org.sciborgs1155.robot.drivetrain;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.drivetrain.DriveConstants.MOI;
import static org.sciborgs1155.robot.drivetrain.DriveConstants.REDUCTION;
import static org.sciborgs1155.robot.drivetrain.DriveConstants.ROBOT_MASS;
import static org.sciborgs1155.robot.drivetrain.DriveConstants.STARTING_POSE;
import static org.sciborgs1155.robot.drivetrain.DriveConstants.TRACK_WIDTH;
import static org.sciborgs1155.robot.drivetrain.DriveConstants.WHEEL_RADIUS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import monologue.Annotations.Log;

/** Simulated {@link DriveIO} class using 4 NEOs */
public class SimDrive implements DriveIO {
  /** Simulated drivetrain with 4 NEO motors */
  private final DifferentialDrivetrainSim simulation =
      new DifferentialDrivetrainSim(
          DCMotor.getNEO(2),
          REDUCTION,
          MOI.in(Kilograms),
          ROBOT_MASS.in(Kilograms),
          WHEEL_RADIUS.in(Meters),
          TRACK_WIDTH.in(Meters),
          VecBuilder.fill(0.00, 0.00, 0.00, 0., 0., 0.00, 0.00)); // Assume sim is 100% accurate

  /**
   * Current Left and Right voltages of the drivetrain (since both have to be updated at once in the
   * 'updatePose' method). These values are updated using the set[Left/Right]Voltage' methods
   */
  private final DifferentialDriveWheelVoltages voltages = new DifferentialDriveWheelVoltages(0, 0);

  /**
   * Displacements of wheels at last odometry update. Similiar usage to the one in {@link
   * SparkDrive}. Used for testing the odometry method in {@link SparkDrive}
   */
  private final DifferentialDriveWheelPositions previousWheelDisplacements =
      new DifferentialDriveWheelPositions(0, 0);

  /** Used for testing the 'updateOdometry' method found in {@link SparkDrive} */
  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(STARTING_POSE.getRotation(), 0, 0, STARTING_POSE);

  /** Angular velocity of the drivetrain(DegreesPerSecond) */
  private double angularVelocity = 0;

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

  @Override
  @Log.NT
  public double getAngularVelocity() {
    return angularVelocity;
  }

  /** NOTE: you can't reset sim encoders (This method does absolutely nothing) */
  @Override
  public void resetEncoders() {}

  /** Used for testing the 'updateOdometry' method found in {@link SparkDrive} */
  public void updateOdometry(double deltaTimeSeconds) {
    // The displacement since last odometry update(as opposed to in total)
    double[] deltaDisplacementsMeters =
        new double[] {
          getLeftDisplacement() - previousWheelDisplacements.leftMeters,
          getRightDisplacement() - previousWheelDisplacements.rightMeters
        };

    // difference in displacement can be used to find a difference in orientation
    double deltaRotationDegrees =
        distanceToAngle(deltaDisplacementsMeters[1] - deltaDisplacementsMeters[0]);

    // old rotation + delta rotation = new rotation
    double newRotation = getPose().getRotation().getDegrees() + deltaRotationDegrees;

    // Rotation has to be calculated in order to use 'DifferentialDriveOdometry'
    odometry.update(
        Rotation2d.fromDegrees(newRotation), getLeftDisplacement(), getRightDisplacement());

    // angular velocity = delta rotation / delta time
    angularVelocity = deltaRotationDegrees / deltaTimeSeconds;

    // Sets up displacements for next update
    previousWheelDisplacements.leftMeters = getLeftDisplacement();
    previousWheelDisplacements.rightMeters = getRightDisplacement();
  }

  /** Used for testing the 'updateOdometry' method found in {@link SparkDrive} */
  @Log.NT
  public Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }

  @Override
  public void updatePose(double deltaTime) {
    simulation.setInputs(voltages.left, voltages.right);
    simulation.update(deltaTime);
    updateOdometry(deltaTime);
  }

  @Override
  public Pose2d getPose() {
    return simulation.getPose();
  }

  public SimDrive() {
    simulation.setPose(STARTING_POSE);
  }
}
