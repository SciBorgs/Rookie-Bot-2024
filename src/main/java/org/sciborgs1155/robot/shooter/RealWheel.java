package org.sciborgs1155.robot.shooter;

import static org.sciborgs1155.robot.Ports.Wheels.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class RealWheel implements WheelIO {
  private final CANSparkMax motor = new CANSparkMax(0, MotorType.kBrushless);
  private final RelativeEncoder encoder;

  public RealWheel() {
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(false);
    motor.setSmartCurrentLimit(30);
    motor.burnFlash();
    encoder = motor.getEncoder();
  }

  @Override
  public double getVelocityRad() {
    return encoder.getVelocity();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }
}
