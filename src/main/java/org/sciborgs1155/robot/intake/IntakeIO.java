package org.sciborgs1155.robot.intake;

public interface IntakeIO {
  /**
   * @return The position of the wrist.
   */
  public double getPosition();

  /**
   * @return The velocity of the wrist.
   */
  public double getVelocity();

  /**
   * Sets the voltage of the wrist.
   * @param volts
   */
  public void setWristVoltage(double volts);
}
