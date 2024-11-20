package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;

public class ShooterConstants {

  public static final double WHEEL_VELOCITY = 2; // TODO: (placeholder)
  public static final double WHEEL_RADIUS = Inches.of(2).in(Meters); // meters (placeholder)
  public static final double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2 * Math.PI;
  public static final double POSITION_FACTOR = WHEEL_CIRCUMFERENCE;
  public static final double VELOCITY_FACTOR = POSITION_FACTOR / 60;

  public static final class Top {
    public static final double kp = 0;
    public static final double ki = 0;
    public static final double kd = 0;
    public static final double ks = 0;
    public static final double kv = 1;
    public static final double ka = 0.01;

    public static final double GEARING = 1;
  }

  public static final class Bottom {
    public static final double kp = 0;
    public static final double ki = 0;
    public static final double kd = 0.001;
    public static final double ks = 0;
    public static final double kv = 1;
    public static final double ka = 0.01;

    public static final double GEARING = 1;
  }
}
