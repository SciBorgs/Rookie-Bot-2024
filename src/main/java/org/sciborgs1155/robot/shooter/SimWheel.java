package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Constants.PERIOD;

import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import monologue.Annotations.Log;

/** SimWheel */
public class SimWheel implements WheelIO {
  @Log.NT FlywheelSim sim;

  public SimWheel() {
    sim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(Top.kv, Top.ka), DCMotor.getNeoVortex(1), 1);
  }

  @Override
  public double getVelocityRad() {
    return sim.getAngularVelocityRadPerSec();
  }

  public void setVoltage(double voltage) {
    sim.setInputVoltage(voltage);
    sim.update(PERIOD.in(Units.Seconds));
  }
}
