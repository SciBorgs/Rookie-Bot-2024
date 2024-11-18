package org.sciborgs1155.robot.shooter;

public class ShooterConstants {

  public static final double WHEEL_VELOCITY = 30; // TODO: (placeholder)
  public static final double WHEEL_RADIUS = 0.05; // meters (placeholder)
  public static final double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2 * Math.PI;
  public static final double POSITION_FACTOR = WHEEL_CIRCUMFERENCE;
  public static final double VELOCITY_FACTOR = POSITION_FACTOR / 60;

  public static final class Top {
    public static final double kp = 1;
    public static final double ki = 1;
    public static final double kd = 1;
    public static final double ks = 1;
    public static final double kv = 1;
    public static final double ka = 1;

    public static final double GEARING = 1;
  }

  public static final class Bottom {
    public static final double kp = 1;
    public static final double ki = 1;
    public static final double kd = 1;
    public static final double ks = 1;
    public static final double kv = 1;
    public static final double ka = 1;

    public static final double GEARING = 1;
  }
}
