package org.sciborgs1155.robot.climber;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public interface ClimberIO {
  public void setVoltage(Measure<Voltage> volts);

  public Measure<Distance> position();

  public Measure<Voltage> volts();

  public Measure<Velocity<Distance>> velocity();
}
