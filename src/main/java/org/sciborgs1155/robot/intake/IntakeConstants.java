package org.sciborgs1155.robot.intake;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Mult;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import edu.wpi.first.units.Velocity;

public class IntakeConstants {

  public static final double WRIST_P = 10;

  public static final double WRIST_I = 0;

  public static final double WRIST_D = 10;

  public static final double WRIST_DOWN = 0;

  /**
   * The factors by which encoder measurements are different from actual motor rotation; default
   * units.
   */
  public static final double MOTOR_GEARING = 4 / 1;

  /** The arm's moment of inertia; resistance to rotational movement. */
  public static final Measure<Mult<Mult<Distance, Distance>, Mass>> MOI =
      (Meters).mult(Meters).mult(Kilograms).of(0.17845);

  public static final Measure<Angle> POSITION_TOLERANCE = Degrees.of(0.8);

  public static final Measure<Mass> MASS = Pounds.of(16);
  public static final Measure<Distance> LENGTH = Inches.of(17.16320);

  public static final Measure<Velocity<Angle>> MAX_VELOCITY = RadiansPerSecond.of(3);
  public static final Measure<Velocity<Velocity<Angle>>> MAX_ACCEL =
      RadiansPerSecond.per(Second).of(4);

  public static final Measure<Angle> STARTING_ANGLE = Degrees.of(99.5);

  public static final Measure<Angle> MIN_ANGLE = Degrees.of(0);
  public static final Measure<Angle> MAX_ANGLE = Degrees.of(99.5);

  public static final double WRIST_UP = MAX_ANGLE.in(Radians);

  public static final double kS = 0.14296;
  public static final double kV = 1.7305;
  public static final double kA = 0.01;
  public static final double kG = 0.12055;
}
