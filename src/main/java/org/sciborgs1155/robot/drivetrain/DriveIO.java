package org.sciborgs1155.robot.drivetrain;

import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.drivetrain.DriveConstants.MAX_VOLTAGE;

import monologue.Logged;

/** Hardware interface for the drivetrain */
public interface DriveIO extends Logged {
  /** Feeds voltage to the left side of the drivetrain(Volts) */
  public void setLeftVoltage(double volts);

  /** Displacement of the left side of the drivetrain(Meters) */
  public double getLeftDisplacement();

  /** Velocity of the left side of the drivetrain(MetersPerSecond) */
  public double getLeftVelocity();

  /** Resets displacement measurement of the left side of the drivetrain */
  public void resetLeftEncoder();

  /** Feeds voltage to the right side of the drivetrain(Volts) */
  public void setRightVoltage(double volts);

  /** Displacement of the right side of the drivetrain(Meters) */
  public double getRightDisplacement();

  /** Velocity of the right side of the drivetrain(MetersPerSecond) */
  public double getRightVelocity();

  /** Resets displacement measurement of the right side of the drivetrain */
  public void resetRightEncoder();

  /** Specifically for sim(since sim needs an update method) */
  default void update() {}

  /** Resets displacement measurement of both sides of the drivetrain */
  default void resetEncoders() {
    resetLeftEncoder();
    resetRightEncoder();
  }

  /**
   * If voltage magnitude is greater than maximum voltage, decreases voltage magnitude so the motors
   * don't explode (voltage is in volts)
   */
  default double clampVoltage(double volts) {
    if (Math.abs(volts) > MAX_VOLTAGE.in(Volts)) {
      return Math.copySign(MAX_VOLTAGE.in(Volts), volts);
    }
    return volts;
  }
}
