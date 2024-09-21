package org.sciborgs1155.robot.climber;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

/** NoClimber */
public class NoClimber implements ClimberIO {

  @Override
  public void setVoltage(Measure<Voltage> volts) {}

  @Override
  public Measure<Distance> position() {
    return Meters.of(0);
  }

  @Override
  public Measure<Voltage> volts() {
    return Volts.of(0);
  }

  @Override
  public Measure<Velocity<Distance>> velocity() {
    return MetersPerSecond.of(0);
  }
}
