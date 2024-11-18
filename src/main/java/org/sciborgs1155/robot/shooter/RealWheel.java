package org.sciborgs1155.robot.shooter;

import static org.sciborgs1155.robot.Ports.Wheels.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.POSITION_FACTOR;
import static org.sciborgs1155.robot.shooter.ShooterConstants.VELOCITY_FACTOR;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class RealWheel implements WheelIO {
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  public RealWheel(boolean inverted, int motorID) {
    motor = new CANSparkMax(motorID, MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(inverted);
    motor.setSmartCurrentLimit(30);
    motor.burnFlash();
    encoder = motor.getEncoder();
    encoder.setPositionConversionFactor(POSITION_FACTOR);
    encoder.setVelocityConversionFactor(VELOCITY_FACTOR);
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }
}
