package org.sciborgs1155.robot.climber;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.climber.ClimbConstants.GEARING;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** SimClimber */
public class SimClimber implements ClimberIO {
  private Measure<Voltage> volts;

  ElevatorSim sim =
      new ElevatorSim(
          LinearSystemId.createElevatorSystem(
              DCMotor.getKrakenX60(2), 2, Units.inchesToMeters(2), GEARING),
          DCMotor.getKrakenX60(2),
          0,
          Units.inchesToMeters(36),
          true,
          0);

  @Override
  public void setVoltage(Measure<Voltage> volts) {
    this.volts = volts;
    sim.setInputVoltage(volts.in(Volts));
    sim.update(PERIOD.in(Seconds));
  }

  @Override
  public Measure<Distance> position() {
    return Meters.of(sim.getPositionMeters());
  }

  @Override
  public Measure<Voltage> volts() {
    return volts;
  }

  @Override
  public Measure<Velocity<Distance>> velocity() {
    return MetersPerSecond.of(sim.getVelocityMetersPerSecond());
  }
}
