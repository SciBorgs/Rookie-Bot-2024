package org.sciborgs1155.robot.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import monologue.Annotations.Log;

import static org.sciborgs1155.robot.Constants.PERIOD;
import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkMax;

/**
 * SimWheel
 */
public class SimWheel implements WheelIO {
  // private final CANSparkMax motor = new CANSparkMax(0, MotorType.kBrushless);
  // private final RelativeEncoder encoder;

  @Log.NT
  DCMotorSim sim;

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

  @Override
  public Command setVoltage(double voltage) {
    return Commands.run(() -> voltageSet(voltage));
  }

  public void voltageSet(double voltage) {
    sim.setInputVoltage(voltage);
    sim.update(PERIOD.in(Units.Seconds));
  }
}