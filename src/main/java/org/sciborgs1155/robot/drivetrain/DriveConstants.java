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

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import monologue.Annotations.Log;
import monologue.Logged;

/** Constants for differential drivetrain. */
public final class DriveConstants {
  public static final Measure<Velocity<Distance>> MAX_SPEED = MetersPerSecond.of(6.4);
  public static final Measure<Velocity<Angle>> MAX_ANGULAR_SPEED = RadiansPerSecond.of(2 * Math.PI);
  public static final Measure<Velocity<Velocity<Distance>>> MAX_ACCEL =
      MetersPerSecondPerSecond.of(8);
  public static final Measure<Velocity<Velocity<Angle>>> MAX_ANGULAR_ACCEL =
      RadiansPerSecond.per(Second).of(2);

  /** Mass per meter squared */
  public static final Measure<Mass> MOMENT_OF_INERTIA = Kilograms.of(7.5);

  public static final Measure<Mass> ROBOT_MASS = Kilograms.of(60);

  public static final Measure<Distance> TRACK_WIDTH = Meters.of(0.75);
  public static final Measure<Distance> WHEEL_BASE = Meters.of(0.75);
  public static final Measure<Distance> WHEEL_RADIUS = Inches.of(3);

  /** Wrapper for the {@link ProfiledPIDController} constants used for driving */
  public static final class DrivePID {
    public static final double PROPORTIONAL_COEFFICIENT = 1.0;
    public static final double INTEGRAL_COEFFICIENT = 0.0;
    public static final double DERIVATIVE_COEFFICIENT = 0.0;

    /** Error tolerance */
    public static final Measure<Distance> POSITION_TOLERANCE = Meters.of(0.01);

    /** Maximum velocity for ending drive commands */
    public static final Measure<Velocity<Distance>> VELOCITY_TOLERANCE = MetersPerSecond.of(0.02);

    /** Configured {@link ProfiledPIDController} */
    public static ProfiledPIDController getController() {
      ProfiledPIDController controller =
          new ProfiledPIDController(
              PROPORTIONAL_COEFFICIENT,
              INTEGRAL_COEFFICIENT,
              DERIVATIVE_COEFFICIENT,
              DRIVE_CONSTRAINTS);
      controller.setTolerance(
          POSITION_TOLERANCE.in(Meters), VELOCITY_TOLERANCE.in(MetersPerSecond));
      return controller;
    }
  }

  /** Wrapper for the {@link ProfiledPIDController} constants used for rotating */
  public static final class RotationPID {
    public static final double PROPORTIONAL_COEFFICIENT = 1.0;
    public static final double INTEGRAL_COEFFICIENT = 0.0;
    public static final double DERIVATIVE_COEFFICIENT = 0.0;

    /** Error tolerance */
    public static final Measure<Angle> POSITION_TOLERANCE = Degrees.of(0.01);

    /** Maximum velocity for ending rotation commands */
    public static final Measure<Velocity<Angle>> VELOCITY_TOLERANCE = DegreesPerSecond.of(0.02);

    /** Configured {@link ProfiledPIDController} */
    public static ProfiledPIDController getController() {
      ProfiledPIDController controller =
          new ProfiledPIDController(
              PROPORTIONAL_COEFFICIENT,
              INTEGRAL_COEFFICIENT,
              DERIVATIVE_COEFFICIENT,
              ROTATION_CONSTRAINTS);
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
     * Configured {@link SimpleMotorFeedforward} controller (input MetersPerSecond, outputs voltage)
     */
    public static SimpleMotorFeedforward getController() {
      return new SimpleMotorFeedforward(S, V, A);
    }
  }

  /** Wrapper for the {@link SimpleMotorFeedforward} constants used for rotating */
  public static final class RotationFeedForward {
    public static final double S = 0.0;
    public static final double V = 0.1;
    public static final double A = 0.01;

    /**
     * Configured {@link SimpleMotorFeedforward} controller (input DegreesPerSecond, outputs
     * voltage)
     */
    public static SimpleMotorFeedforward getController() {
      return new SimpleMotorFeedforward(S, V, A);
    }
  }

  /**
   * If voltage magnitude is greater than maximum voltage, decreases voltage magnitude (So the
   * motors don't explode :)
   */
  public static Measure<Voltage> clampVoltage(Measure<Voltage> voltage) {
    if (Math.abs(voltage.in(Volts)) > MAX_VOLTAGE.in(Volts)) {
      return Volts.of(Math.copySign(MAX_VOLTAGE.in(Volts), voltage.in(Volts)));
    }
    return voltage;
  }

  /** Converts linear displacement of motors to angular distances of drivetrain */
  public static Measure<Angle> distanceToAngle(Measure<Distance> distance) {
    return Degrees.of(distance.in(Meters) / WHEEL_BASE.times(Math.PI).divide(360).in(Meters));
  }

  /** Maximum voltage able to be (safely) fed to motors */
  public static final Measure<Voltage> MAX_VOLTAGE = Volts.of(12);

  /** Trapazoidal motion profile constraints for {@link ProfiledPIDController} */
  public static final Constraints DRIVE_CONSTRAINTS =
      new Constraints(MAX_SPEED.in(MetersPerSecond), MAX_ACCEL.in(MetersPerSecondPerSecond));

  /** Trapazoidal motion profile constraints for {@link ProfiledPIDController} */
  public static final Constraints ROTATION_CONSTRAINTS =
      new Constraints(
          MAX_ANGULAR_SPEED.in(DegreesPerSecond),
          MAX_ANGULAR_ACCEL.in(DegreesPerSecond.per(Second)));

  /** Gearing reduction ratio. */
  public static final double REDUCTION = 7.21;

  /** Translation: heading : Velocity(L and R) : Position(L and R) */
  public static final Vector<N7> STANDARD_MEASUREMENT_DEVIATIONS =
      VecBuilder.fill(0.00, 0.00, 0.00, 0., 0., 0.00, 0.00);

  /** Array of motor ID's(for cleaner-looking instantiation) */
  public static final int[] MOTOR_IDS =
      new int[] {FRONT_LEFT_DRIVE, REAR_LEFT_DRIVE, FRONT_RIGHT_DRIVE, REAR_RIGHT_DRIVE};

  /** Initial position of the robot */
  public static final Pose2d STARTING_POSE =
      new Pose2d(Meters.of(5), Meters.of(5), Rotation2d.fromDegrees(0));

  /** Allows for precision movement */
  public static final double SLOW_SPEED = 0.33;

  /** Allows for quicker, broad movements */
  public static final double FULL_SPEED = 1.0;

  /** Magnitude of joystick input to be considered negligable */
  public static final double DEADBAND = 0.1;

  /** Stores and logs essential drivetrain data */
  public static class DiffDrivetrainLogger implements Logged {
    /** Position setpoints for autonomous commands */
    @Log.NT public Pose2d goalPose;

    @Log.NT public Pose2d pose = STARTING_POSE;

    @Log.NT public double targetLeftVoltage = 0;

    @Log.NT public double targetRightVoltage = 0;

    /** MetersPerSecond */
    @Log.NT public double leftVelocity = 0;

    /** MetersPerSecond */
    @Log.NT public double rightVelocity = 0;

    /** DegreesPerSecond */
    @Log.NT public double angularVelocity = 0;

    /** GUI displaying robot pose */
    @Log.NT public Field2d field = new Field2d();
  }
}
