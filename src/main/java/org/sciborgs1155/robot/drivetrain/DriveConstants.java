package org.sciborgs1155.robot.drivetrain;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

/** Constants for differential drivetrain */
public final class DriveConstants {
  /** Moment of Inertia mass(per meter squared) */
  public static final Measure<Mass> MOI = Kilograms.of(7.5);

  /** Mass of the robot(not having this javadoc gives me pain) */
  public static final Measure<Mass> ROBOT_MASS = Kilograms.of(60);

  /** Distance between left and right wheels */
  public static final Measure<Distance> TRACK_WIDTH = Meters.of(0.75);

  /** Distance between front and back wheels */
  public static final Measure<Distance> WHEEL_BASE = Meters.of(0.75);

  /** Radius of one wheel */
  public static final Measure<Distance> WHEEL_RADIUS = Inches.of(3);

  /** Gearing reduction ratio */
  public static final double REDUCTION = 7.21;

  /** Allows for precision movement */
  public static final double SLOW_SPEED = 0.33;

  /** Allows for quicker, broad movements */
  public static final double FULL_SPEED = 1.0;

  /** Magnitude of joystick input to be considered negligable */
  public static final double DEADBAND = 0.02;

  /** Maximum voltage able to be (safely) fed to motors */
  public static final Measure<Voltage> MAX_VOLTAGE = Volts.of(12);

  /** Initial position of the robot(fed into odometry classes) */
  public static final Pose2d STARTING_POSE =
      new Pose2d(Meters.of(5), Meters.of(5), Rotation2d.fromDegrees(0));
}
