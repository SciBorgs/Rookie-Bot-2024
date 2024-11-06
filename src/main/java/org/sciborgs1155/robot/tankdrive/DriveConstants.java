package org.sciborgs1155.robot.tankdrive;

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
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

/** Constants for differential drivetrain. */
public final class DriveConstants {
  /** Max speed of a motor. */
  public static final Measure<Velocity<Distance>> MAX_SPEED = MetersPerSecond.of(6.4);

  /** Max turning speed of the drivetrain. */
  public static final Measure<Velocity<Angle>> MAX_ANGULAR_SPEED = RadiansPerSecond.of(2 * Math.PI);

  /** Mass of the robot. */
  public static final Measure<Mass> ROBOT_MASS = Kilograms.of(60);

  /** Moment of inertia(per meter squared). */
  public static final Measure<Mass> MOI_MASS = Kilograms.of(7.5);

  /** Max acceleration of a motor. */
  public static final Measure<Velocity<Velocity<Distance>>> MAX_ACCEL = MetersPerSecondPerSecond.of(8);

  /** Max turning acceleration of the drivetrain. */
  public static final Measure<Velocity<Velocity<Angle>>> MAX_ANGULAR_ACCEL = RadiansPerSecond.per(Second)
      .of(2);

  /** Distance between right and left wheels on robot */
  public static final Measure<Distance> TRACK_WIDTH = Meters.of(0.75);

  /** Distance between front and back wheels on robot */
  public static final Measure<Distance> WHEEL_BASE = Meters.of(0.75);

  /** Radius of the wheels. */
  public static final Measure<Distance> WHEEL_RADIUS = Inches.of(3);

  /** Translation PID constants. */
  public static final class TranslationPID {
    /** Proportional coefficient. */
    public static final double P = 10;

    /** Integral coefficient. */
    public static final double I = 0.0;

    /** Derivative coefficient. */
    public static final double D = 0.5;

    /** Error Tolerance. */
    public static final Measure<Distance> TOLERANCE = Meters.of(0.1);

    /**
     * PID controller for translation(outputs MetersPerSecond).
     */
    public static ProfiledPIDController getController() {
      ProfiledPIDController controller = new ProfiledPIDController(P, I, D, DRIVE_CONSTRAINTS);
      controller.setTolerance(TOLERANCE.in(Meters));
      return controller;
    }
  }

  /** Rotation PID constants. */
  public static final class RotationPID {
    /** Proportional coefficient. */
    public static final double P = 1;

    /** Integral coefficient. */
    public static final double I = 0.0;

    /** Derivative coefficient. */
    public static final double D = 0.2;

    /** Position threshold for ending the command. */
    public static final Measure<Angle> POSITION_TOLERANCE = Degrees.of(0.01);

    /** Velocity threshold for ending the command. */
    public static final Measure<Velocity<Angle>> VELOCITY_TOLERANCE = DegreesPerSecond.of(0.1);

    /** PID controller for rotation(outputs DegreesPerSecond). */
    public static ProfiledPIDController getController() {
      ProfiledPIDController controller = new ProfiledPIDController(P, I, D, ROTATION_CONSTRAINTS);
      controller.setTolerance(POSITION_TOLERANCE.in(Degrees), VELOCITY_TOLERANCE.in(DegreesPerSecond));
      controller.setGoal(new State(0, 0));
      return controller;
    }
  }

  /** Translation FeedForward constants. */
  public static final class TranslationFFD {
    /** Static gain. */
    public static final double S = 0.0;

    /** Velocity gain. */
    public static final double V = 0.1;

    /** Acceleration gain. */
    public static final double A = 0.01;

    /** When output is below tolerance, command should be considered done. */
    public static final Measure<Voltage> TOLERANCE = Volts.of(1);

    /** FFD controller for translation(input MetersPerSecond, outputs voltage). */
    public static SimpleMotorFeedforward getController() {
      return new SimpleMotorFeedforward(S, V, A);
    }
  }

  /** Rotation FeedForward constants. */
  public static final class RotationFFD {
    /** Static gain. */
    public static final double S = 0.0;

    /** Velocity gain. */
    public static final double V = 0.1;

    /** Acceleration gain. */
    public static final double A = 0.01;

    /** FFD controller for rotation(input DegreesPerSecond, outputs voltage). */
    public static SimpleMotorFeedforward getController() {
      return new SimpleMotorFeedforward(S, V, A);
    }
  }

  /** Converts linear distances to angular distances. */
  public static Measure<Angle> distanceToAngle(Measure<Distance> distance) {
    return Degrees.of(distance.in(Meters) / WHEEL_BASE.times(Math.PI).divide(360).in(Meters));
  }

  /** Maximum voltage of wheels. */
  public static final Measure<Voltage> MAX_VOLTAGE = Volts.of(12);

  /** Trapazoidal motion profile constraints for translation. */
  public static final Constraints DRIVE_CONSTRAINTS = new Constraints(MAX_SPEED.in(MetersPerSecond),
      MAX_ACCEL.in(MetersPerSecondPerSecond));

  /** Trapazoidal motion profile constraints for rotation. */
  public static final Constraints ROTATION_CONSTRAINTS = new Constraints(
      MAX_ANGULAR_SPEED.in(DegreesPerSecond),
      MAX_ANGULAR_ACCEL.in(DegreesPerSecond.per(Second)));

  /** Gearing reduction ratio. */
  public static final double REDUCTION = 7.21;

  /** Translation: heading : Velocity(L and R) : Position(L and R) */
  public static final Vector<N7> STANDARD_MEASUREMENT_DEVIATIONS = VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005,
      0.005);

  /** Array of motor ID's(for cleaner instantiation). */
  public static final int[] MOTOR_IDS = new int[] { FRONT_LEFT_DRIVE, REAR_LEFT_DRIVE, FRONT_RIGHT_DRIVE,
      REAR_RIGHT_DRIVE };

  /** Starting pose of the robot. */
  public static final Pose2d STARTING_POSE = new Pose2d(Meters.of(5), Meters.of(5), Rotation2d.fromDegrees(0));

  /** Allows for precision movement. */
  public static final double SLOW_SPEED = 0.33;

  /** Allows for quicker, broad movements. */
  public static final double FULL_SPEED = 1.0;

  /** Magnitude of joystick input to be considered negligable. */
  public static final double DEADBAND = 0.1;
}
