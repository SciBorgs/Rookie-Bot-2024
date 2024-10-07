package org.sciborgs1155.robot.intake;

public class SimIntake implements IntakeIO {

  public SimIntake() {}

  @Override
  public void setRoller(double speed) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public double getPosition() {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public double getVelocity() {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public void setWristVoltage(double volts) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public void updatePosition(double setpoint) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateSetpoint'");
  }
}
