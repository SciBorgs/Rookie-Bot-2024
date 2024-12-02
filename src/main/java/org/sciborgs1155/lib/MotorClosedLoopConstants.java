package org.sciborgs1155.lib;

public class MotorClosedLoopConstants {
  public final double kS;
  public final double kV;
  public final double kA;
  public final double kP;
  public final double kI;
  public final double kD;
  public final double iZone;
  public final double positionTolerance;
  public final double velocityTolerance;
  public final double maxVelocity;
  public final double maxAcceleration;

  public MotorClosedLoopConstants(
      double kS,
      double kV,
      double kA,
      double kP,
      double kI,
      double kD,
      double izone,
      double positionTolerance,
      double velocityTolerance,
      double maxVelocity,
      double maxAcceleration) {
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.iZone = izone;
    this.positionTolerance = positionTolerance;
    this.velocityTolerance = velocityTolerance;
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
  }
}
