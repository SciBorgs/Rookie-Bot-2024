package org.sciborgs1155.robot.intake;

public interface IntakeIO {

  public void setRoller(double speed);

  public double getPosition();

  public double getVelocity();

  public void setWristVoltage(double volts);

  public void updatePosition(double setpoint);
}
