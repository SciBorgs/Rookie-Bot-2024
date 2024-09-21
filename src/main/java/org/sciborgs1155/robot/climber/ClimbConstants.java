package org.sciborgs1155.robot.climber;

public class ClimbConstants {
  public static final double ROTOR_OFFSET = 0.0;
  public static final double POSITION_CONVERSION = 1.0;
  public static final double GEARING = 30;

  // Gains
  public static final double kP = 25.589; // 25.589? P,//0.09266 V, 0.00074699 V
  public static final double kI = 0;
  public static final double kD = 0; // 0.306? P, 0 V

  public static final double kS = 0;
  public static final double kG = 0.028085;
  public static final double kV = 11.217;
  public static final double kA = 0.13881;

  public static final double MAX_VELO = 1;
  public static final double MAX_ACCEL = 1;
}
