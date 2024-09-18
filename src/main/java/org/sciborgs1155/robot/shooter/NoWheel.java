package org.sciborgs1155.robot.shooter;

public class NoWheel implements WheelIO {
    @Override
    public void setVoltage(double voltage) {}

    @Override
    public double getSpeed() {
        return 0;
    }
}
