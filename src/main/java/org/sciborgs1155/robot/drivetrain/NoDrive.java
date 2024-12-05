package org.sciborgs1155.robot.drivetrain;

/** Disfunctional Placeholder {@link DriveIO} class */
public class NoDrive implements DriveIO {
  @Override
  public void setLeftVoltage(double volts) {}

  @Override
  public double getLeftDisplacement() {
    return 0;
  }

  @Override
  public double getLeftVelocity() {
    return 0;
  }

  @Override
  public void resetLeftEncoder() {}

  @Override
  public void setRightVoltage(double volts) {}

  @Override
  public double getRightDisplacement() {
    return 0;
  }

  @Override
  public double getRightVelocity() {
    return 0;
  }

  @Override
  public void resetRightEncoder() {}

  @Override
  public void resetEncoders() {}
}
