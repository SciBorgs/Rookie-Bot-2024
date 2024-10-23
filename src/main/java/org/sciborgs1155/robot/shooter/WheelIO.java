package org.sciborgs1155.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;

public interface WheelIO {
  
  public void setVoltage(double voltage);
  public double getVelocityRad();
}
