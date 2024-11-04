package org.sciborgs1155.robot.shooter;

/** NoWheel */
public class NoWheel implements WheelIO {

  @Override
  public void setVoltage(double volts) {}

  @Override
  public double getVelocityRad() {
    return 0;
  }
}
