package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public final class DriveConstants {
  /** Max speed of a motor. */
  public static final Measure<Velocity<Distance>> MAX_SPEED = MetersPerSecond.of(5.74);

  /** Max turning speed of the drivetrain. */
  public static final Measure<Velocity<Angle>> MAX_ANGULAR_SPEED = RadiansPerSecond.of(2 * Math.PI);

  /** Robot mass. */
  public static final Measure<Mass> ROBOT_MASS = Kilograms.of(60);

  /** Moment of inertia(per meter squared). */
  public static final Measure<Mass> MOI_MASS = Kilograms.of(7.5);

  /** Max acceleration of a motor. */
  public static final Measure<Velocity<Velocity<Distance>>> MAX_ACCEL = MetersPerSecondPerSecond.of(8);

  /** Max turning acceleration of the drivetrain. */
  public static final Measure<Velocity<Velocity<Angle>>> MAX_ANGULAR_ACCEL = RadiansPerSecond.per(Second)
      .of(2 * Math.PI);

  /** Distance between centers of right and left wheels on robot */
  public static final Measure<Distance> TRACK_WIDTH = Meters.of(0.5715);

  /** Distance between front and back wheels on robot */
  public static final Measure<Distance> WHEEL_BASE = Meters.of(0.5715);

  /** Radius of wheels. */
  public static final Measure<Distance> WHEEL_RADIUS = Inches.of(3);

  /** Positions of the modules relative to the orgin of the robot. */
  public static final Translation2d[] MODULE_OFFSET = {
      new Translation2d(WHEEL_BASE.divide(2), TRACK_WIDTH.divide(2)), // front
      // left
      new Translation2d(WHEEL_BASE.divide(2), TRACK_WIDTH.divide(-2)), // front right
      new Translation2d(WHEEL_BASE.divide(-2), TRACK_WIDTH.divide(2)), // rear left
      new Translation2d(WHEEL_BASE.divide(-2), TRACK_WIDTH.divide(-2)) // rear right
  };

  /** PID constants used for driving. */
  public static final class DrivePID {
    public static final double P = 0.6;
    public static final double I = 0.0;
    public static final double D = 0.0;
  }

  /** PID constants used for rotating. */
  public static final class RotationPID {
    public static final double P = 0.6;
    public static final double I = 0.0;
    public static final double D = 0.0;
  }

  /** FFD constants used for driving. */
  public static final class DriveFFD {
    public static final double S = 0.0;
    public static final double V = 0.1;
    public static final double A = 0.01;
  }

  /** FFD constants used for rotation. */
  public static final class RotationFFD {
    public static final double S = 0.0;
    public static final double V = 0.1;
    public static final double A = 0.01;
  }

  /**
   * Converts a distance(traveled by one side of the drivetrain) to an angular
   * distance.
   */
  public static final Measure<Angle> distanceToAngle(Measure<Distance> distance) {
    return Radians.of(distance.divide(TRACK_WIDTH.times(2).times(Math.PI).in(Meters)).in(Meters));
  }

  /** Maximum voltage of wheels. */
  public static final Measure<Voltage> MAX_VOLTAGE = Volts.of(12);

  /** Constraints for driving. */
  public static final Constraints DRIVE_CONSTRAINTS = new Constraints(MAX_SPEED.in(MetersPerSecond),
      MAX_ACCEL.in(MetersPerSecondPerSecond));

  /** Constraints for rotation. */
  public static final Constraints ROTATION_CONSTRAINTS = new Constraints(
      MAX_ANGULAR_SPEED.in(RadiansPerSecond),
      MAX_ANGULAR_ACCEL.in(RadiansPerSecond.per(Second)));
  
  /** Gearing reduction. */
  public static final double REDUCTION = 7.21;

  /** Measurement deviations. */
  // x and y: 0.001 m
  // heading: 0.001 rad
  // l and r velocity: 0.1 m/s
  // l and r position: 0.005 m
  public static final Vector<N7> STD_DEVS = VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005);
}
