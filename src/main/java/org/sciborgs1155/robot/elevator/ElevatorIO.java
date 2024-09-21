package org.sciborgs1155.robot.elevator;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

/** ElevatorIO */
public interface ElevatorIO {
  /* Sets input voltage. */
  public void setVoltage(Measure<Voltage> volts);

  /* Gets voltage set. */
  public Measure<Voltage> volts();

  /* Gets current height in Meters. */
  public Measure<Distance> currentHeight();

  /* Gets current velocity in MetersPerSecond. */
  public Measure<Velocity<Distance>> currentVelo();
}
