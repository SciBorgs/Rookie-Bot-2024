package org.sciborgs1155.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * NoWheel
 */
public class NoWheel implements WheelIO {

    @Override
    public void setVoltage(double volts) {}

    @Override
    public double getVelocityRad() {
        return 0;
    }

    
}