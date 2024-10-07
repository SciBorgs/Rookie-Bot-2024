package org.sciborgs1155.robot.intake;

public interface IntakeIO {
    public void setRoller(double speed);
    public double getPositon();
    public double getVelocity();
    public void setWrist(double volts);
}
