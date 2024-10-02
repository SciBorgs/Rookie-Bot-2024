package org.sciborgs1155.robot.climber;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public class ClimbConstants {
  public static final double ROTOR_OFFSET = 0.0;
  public static final double POSITION_CONVERSION = 1.0;
  public static final double GEARING = 30;

  // Gains
  public static final double kP = 25.232; // 25.589? P,//0.09266 V, 0.00074699 V
  public static final double kI = 0;
  public static final double kD = 0.29529; // 0.306? P, 0 V

  public static final double kS = 0;
  public static final double kG = 0.055313;
  public static final double kV = 9.8053;
  public static final double kA = 0.12357;

  public static final double MAX_VELO = 1;
  public static final double MAX_ACCEL = 2;

  public class Sim {
    public static final Measure<Distance> MAX_HEIGHT = Meters.of(Units.inchesToMeters(36));
    public static final Measure<Distance> MIN_HEIGHT = Meters.of(0);
    public static final Measure<Distance> START_HEIGHT = Meters.of(0);
  }
}
