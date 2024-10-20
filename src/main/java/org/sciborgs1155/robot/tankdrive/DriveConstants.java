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
import static org.sciborgs1155.robot.Ports.Drive.FRONT_LEFT_DRIVE;
import static org.sciborgs1155.robot.Ports.Drive.FRONT_RIGHT_DRIVE;
import static org.sciborgs1155.robot.Ports.Drive.REAR_LEFT_DRIVE;
import static org.sciborgs1155.robot.Ports.Drive.REAR_RIGHT_DRIVE;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
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
  public static final Measure<Velocity<Velocity<Distance>>> MAX_ACCEL =
      MetersPerSecondPerSecond.of(8);

  /** Max turning acceleration of the drivetrain. */
  public static final Measure<Velocity<Velocity<Angle>>> MAX_ANGULAR_ACCEL =
      RadiansPerSecond.per(Second).of(2 * Math.PI);

  /** Distance between right and left wheels on robot */
  public static final Measure<Distance> TRACK_WIDTH = Meters.of(0.5715);

  /** Distance between front and back wheels on robot */
  public static final Measure<Distance> WHEEL_BASE = Meters.of(0.5715);

  /** Radius of wheels. */
  public static final Measure<Distance> WHEEL_RADIUS = Inches.of(3);

  /** PID constants used for driving. */
  public static final class DrivePID {
    /** Proportional coefficient. */
    public static final double P = 10;

    /** Integral coefficient. */
    public static final double I = 0.0;

    /** Derivative coefficient. */
    public static final double D = 0.5;

    /** Error Tolerance(Meters). */
    public static final double TOLERANCE = 0.2;

    /** PID controller for driving(linear error -> linear velocity). */
    public static ProfiledPIDController getController() {
      ProfiledPIDController controller = new ProfiledPIDController(P, I, D, ROTATION_CONSTRAINTS);
      controller.setTolerance(TOLERANCE);
      return controller;
    }
  }

  /** PID constants used for rotating. */
  public static final class RotationPID {
    /** Proportional coefficient. */
    public static final double P = 1;

    /** Integral coefficient. */
    public static final double I = 0.0;

    /** Derivative coefficient. */
    public static final double D = 0.5;

    /** Error Tolerance(Radians). */
    public static final double TOLERANCE = 0.008;

    /** PID controller for rotating(angle error -> angule velocity). */
    public static ProfiledPIDController getController() {
      ProfiledPIDController controller = new ProfiledPIDController(P, I, D, ROTATION_CONSTRAINTS);
      controller.setTolerance(TOLERANCE);
      return controller;
    }
  }

  /** FFD constants used for driving. */
  public static final class DriveFFD {
    /** Static gain. */
    public static final double S = 0.0;

    /** Velocity gain. */
    public static final double V = 0.1;

    /** Acceleration gain. */
    public static final double A = 0.01;

    /** FFD controller for driving(velocity -> voltage). */
    public static SimpleMotorFeedforward getController() {
      return new SimpleMotorFeedforward(S, V, A);
    }
  }

  /** FFD constants used for rotation. */
  public static final class RotationFFD {
    /** Static gain. */
    public static final double S = 0.0;

    /** Velocity gain. */
    public static final double V = 0.1;

    /** Acceleration gain. */
    public static final double A = 0.01;

    /** FFD controller for rotating(drivetrain angular velocity ->voltage). */
    public static SimpleMotorFeedforward getController() {
      return new SimpleMotorFeedforward(S, V, A);
    }
  }

  /** Converts a distance(traveled by one side of the drivetrain) to an angular distance. */
  public static final Measure<Angle> distanceToAngle(Measure<Distance> distance) {
    return Radians.of(distance.divide(TRACK_WIDTH.times(2).times(Math.PI).in(Meters)).in(Meters));
  }

  /** Maximum voltage of wheels. */
  public static final Measure<Voltage> MAX_VOLTAGE = Volts.of(12);

  /** Constraints for drive PID and FFD. */
  public static final Constraints DRIVE_CONSTRAINTS =
      new Constraints(MAX_SPEED.in(MetersPerSecond), MAX_ACCEL.in(MetersPerSecondPerSecond));

  /** Constraints for rotation PID and FFD. */
  public static final Constraints ROTATION_CONSTRAINTS =
      new Constraints(
          MAX_ANGULAR_SPEED.in(RadiansPerSecond),
          MAX_ANGULAR_ACCEL.in(RadiansPerSecond.per(Second)));

  /** Gearing reduction ratio. */
  public static final double REDUCTION = 7.21;

  /**
   * Measurement deviations. x and y: 0.001 m: heading: 0.001 rad: l and r velocity: 0.1 m/s: l and
   * r position: 0.005 m:
   */
  public static final Vector<N7> STD_DEVS =
      VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005);

  /** Array of motor ID's(for cleaner instantiation). */
  public static final int[] MOTOR_IDS =
      new int[] {FRONT_LEFT_DRIVE, REAR_LEFT_DRIVE, FRONT_RIGHT_DRIVE, REAR_RIGHT_DRIVE};

  /** Starting pose of the robot. */
  public static final Pose2d STARTING_POSE = new Pose2d();
}
