package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Constants.PERIOD;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import monologue.Annotations.Log;

/** SimWheel */
public class SimWheel implements WheelIO {
  // private final CANSparkMax motor = new CANSparkMax(0, MotorType.kBrushless);
  // private final RelativeEncoder encoder;

  @Log.NT DCMotorSim sim;

  public SimWheel() {
    // motor.setIdleMode(IdleMode.kBrake);
    // motor.setInverted(false);
    // motor.setSmartCurrentLimit(30);
    // motor.burnFlash();
    // encoder = motor.getEncoder();
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
