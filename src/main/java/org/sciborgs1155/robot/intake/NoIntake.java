package org.sciborgs1155.robot.intake;

public class NoIntake implements IntakeIO {

  @Override
  public void setRoller(double speed) {}

  @Override
  public double getPosition() {
    return 0;
  }

  @Override
  public double getVelocity() {
    return 0;
  }

  @Override
  public void setWristVoltage(double volts) {}

  @Override
  public void updatePosition(double setpoint) {}
}
