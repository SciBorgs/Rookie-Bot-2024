package org.sciborgs1155.robot.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Ports.Drive.FRONT_LEFT_DRIVE;
import static org.sciborgs1155.robot.Ports.Drive.FRONT_RIGHT_DRIVE;
import static org.sciborgs1155.robot.Ports.Drive.REAR_LEFT_DRIVE;
import static org.sciborgs1155.robot.Ports.Drive.REAR_RIGHT_DRIVE;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

/** Constants for differential drivetrain */
public final class DriveConstants {
  public static final Measure<Velocity<Distance>> MAX_SPEED = MetersPerSecond.of(6.4);
  public static final Measure<Velocity<Angle>> MAX_ANGULAR_SPEED = RadiansPerSecond.of(2 * Math.PI);
  public static final Measure<Velocity<Velocity<Distance>>> MAX_ACCEL = MetersPerSecondPerSecond.of(8);
  public static final Measure<Velocity<Velocity<Angle>>> MAX_ANGULAR_ACCEL = RadiansPerSecond.per(Second).of(2);

  /** Moment of Inertia mass(per meter squared) */
  public static final Measure<Mass> MOI = Kilograms.of(7.5);

  public static final Measure<Mass> ROBOT_MASS = Kilograms.of(60);

  public static final Measure<Distance> TRACK_WIDTH = Meters.of(0.75);
  public static final Measure<Distance> WHEEL_BASE = Meters.of(0.75);
  public static final Measure<Distance> WHEEL_RADIUS = Inches.of(3);

  /** Wrapper for the {@link ProfiledPIDController} constants used for driving */
  public static final class DrivePID {
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    /** Error tolerance */
    public static final Measure<Distance> POSITION_TOLERANCE = Meters.of(0.01);

    /** Maximum velocity for ending drive commands */
    public static final Measure<Velocity<Distance>> VELOCITY_TOLERANCE = MetersPerSecond.of(0.02);

    /** Configured {@link ProfiledPIDController} */
    public static ProfiledPIDController getController() {
      ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, DRIVE_CONSTRAINTS);
      controller.setTolerance(
          POSITION_TOLERANCE.in(Meters), VELOCITY_TOLERANCE.in(MetersPerSecond));
      return controller;
    }
  }

  /** Wrapper for the {@link ProfiledPIDController} constants used for rotating */
  public static final class RotationPID {
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    /** Error tolerance */
    public static final Measure<Angle> POSITION_TOLERANCE = Degrees.of(0.01);

    /** Maximum velocity for ending rotation commands */
    public static final Measure<Velocity<Angle>> VELOCITY_TOLERANCE = DegreesPerSecond.of(0.02);

    /** Configured {@link ProfiledPIDController} */
    public static ProfiledPIDController getController() {
      ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, ROTATION_CONSTRAINTS);
      controller.setTolerance(
          POSITION_TOLERANCE.in(Degrees), VELOCITY_TOLERANCE.in(DegreesPerSecond));
      return controller;
    }
  }

  /** Wrapper for the {@link SimpleMotorFeedforward} constants used for driving */
  public static final class DriveFeedForward {
    public static final double S = 0.0;
    public static final double V = 0.1;
    public static final double A = 0.01;

    /**
     * Configured {@link SimpleMotorFeedforward} controller (input MetersPerSecond,
     * outputs voltage)
     */
    public static SimpleMotorFeedforward getController() {
      return new SimpleMotorFeedforward(S, V, A);
    }
  }

  /**
   * Wrapper for the {@link SimpleMotorFeedforward} constants used for rotating
   */
  public static final class RotationFeedForward {
    public static final double S = 0.0;
    public static final double V = 0.1;
    public static final double A = 0.01;

    /**
     * Configured {@link SimpleMotorFeedforward} controller (input DegreesPerSecond,
     * outputs
     * voltage)
     */
    public static SimpleMotorFeedforward getController() {
      return new SimpleMotorFeedforward(S, V, A);
    }
  }

  /** Maximum voltage able to be (safely) fed to motors */
  public static final Measure<Voltage> MAX_VOLTAGE = Volts.of(12);

  /** Trapazoidal motion profile constraints for {@link ProfiledPIDController} */
  public static final Constraints DRIVE_CONSTRAINTS = new Constraints(MAX_SPEED.in(MetersPerSecond),
      MAX_ACCEL.in(MetersPerSecondPerSecond));

  /** Trapazoidal motion profile constraints for {@link ProfiledPIDController} */
  public static final Constraints ROTATION_CONSTRAINTS = new Constraints(
      MAX_ANGULAR_SPEED.in(DegreesPerSecond),
      MAX_ANGULAR_ACCEL.in(DegreesPerSecond.per(Second)));

  /** Gearing reduction ratio */
  public static final double REDUCTION = 7.21;

  /** Array of motor ID's(for cleaner-looking instantiation) */
  public static final int[] MOTOR_IDS = new int[] { FRONT_LEFT_DRIVE, REAR_LEFT_DRIVE, FRONT_RIGHT_DRIVE,
      REAR_RIGHT_DRIVE };

  /** Initial position of the robot */
  public static final Pose2d STARTING_POSE = new Pose2d(Meters.of(5), Meters.of(5), Rotation2d.fromDegrees(0));

  /** Allows for precision movement */
  public static final double SLOW_SPEED = 0.33;

  /** Allows for quicker, broad movements */
  public static final double FULL_SPEED = 1.0;

  /** Magnitude of joystick input to be considered negligable */
  public static final double DEADBAND = 0.1;
}
