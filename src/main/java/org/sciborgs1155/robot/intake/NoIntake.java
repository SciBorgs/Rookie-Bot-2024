package org.sciborgs1155.robot.intake;

public class NoIntake implements IntakeIO {
/**
 * @return 0
 */
  @Override
  public double getPosition() {
    return 0;
  }
/**
 * @return 0
 */
  @Override
  public double getVelocity() {
    return 0;
  }
/**
 * Is useless.
 */
  @Override
  public void setWristVoltage(double volts) {}
}
