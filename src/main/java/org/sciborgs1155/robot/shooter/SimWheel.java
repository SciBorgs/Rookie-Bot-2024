package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** SimWheel */
public class SimWheel implements WheelIO {
  FlywheelSim sim;

  public SimWheel() {
    sim =
        new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(Top.kv, Top.ka), DCMotor.getNeoVortex(1), 1);
  }

  @Override
  public double getVelocity() {
    return sim.getAngularVelocityRadPerSec();
  }

  @Override
  public void setVoltage(double voltage) {
    sim.setInputVoltage(voltage);
    sim.update(PERIOD.in(Seconds));
  }
}
