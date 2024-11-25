package org.sciborgs1155.robot.shooter;

public interface WheelIO {

  /**
   * Sets the voltage.
   *
   * @param voltage The target voltage speed.
   */
  public void setVoltage(double voltage);

  /**
   * Returns the velocity of a wheel.
   *
   * @return the velocity of a wheel.
   */
  public double getVelocity();
}
