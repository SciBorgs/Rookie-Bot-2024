package org.sciborgs1155.robot.elevator;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

/** SimStateSpaceEleavtor */
public class SimElevator implements ElevatorIO {

  @Override
  public void setVoltage(Measure<Voltage> volts) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
  }

  @Override
  public Measure<Voltage> volts() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'volts'");
  }

  @Override
  public Measure<Distance> currentHeight() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'currentHeight'");
  }

  @Override
  public Measure<Velocity<Distance>> currentVelo() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'currentVelo'");
  }
}
