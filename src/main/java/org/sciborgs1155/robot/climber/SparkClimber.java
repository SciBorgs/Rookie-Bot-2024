package org.sciborgs1155.robot.climber;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.Ports.Climber.*;
import static org.sciborgs1155.robot.climber.ClimbConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public class SparkClimber implements ClimberIO {

  CANSparkMax lead = new CANSparkMax(0, MotorType.kBrushless);
  CANSparkMax follower = new CANSparkMax(1, MotorType.kBrushless);

  RelativeEncoder encoder = lead.getEncoder();

  public SparkClimber() {
    follower.follow(lead);

    lead.burnFlash();
    lead.setSmartCurrentLimit(30);
    lead.setIdleMode(IdleMode.kBrake);
    lead.setInverted(false);
    lead.setControlFramePeriodMs(20);

    follower.burnFlash();
    follower.setSmartCurrentLimit(30);
    follower.setIdleMode(IdleMode.kBrake);
    follower.setInverted(true);
    follower.setControlFramePeriodMs(20);

    encoder.setPositionConversionFactor(POSITION_CONVERSION);
    encoder.setVelocityConversionFactor(POSITION_CONVERSION * PERIOD.in(Seconds));
  }

  @Override
  public void setVoltage(Measure<Voltage> volts) {
    lead.setVoltage(volts.in(Volts));
  }

  @Override
  public Measure<Distance> position() {
    return Meters.of(encoder.getPosition());
  }

  @Override
  public Measure<Velocity<Distance>> velocity() {
    return MetersPerSecond.of(encoder.getVelocity());
  }

  @Override
  public Measure<Voltage> volts() {
    return Volts.of(lead.getBusVoltage());
  }
}
