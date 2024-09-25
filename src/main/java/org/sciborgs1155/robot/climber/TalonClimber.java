package org.sciborgs1155.robot.climber;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Ports.Climber.*;
import static org.sciborgs1155.robot.climber.ClimbConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public class TalonClimber implements ClimberIO {

  private TalonFX talon = new TalonFX(talonID);
  private TalonFX follower = new TalonFX(followerID);
  private TalonFXConfigurator configurator;
  private TalonFXConfiguration config = new TalonFXConfiguration();

  public TalonClimber() {
    configurator = talon.getConfigurator();

    // check over whether all of this is correct
    config
        .withCurrentLimits(
            config.CurrentLimits.withStatorCurrentLimit(30)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentLimitEnable(true))
        .withMotorOutput(
            config.MotorOutput.withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.valueOf("Master climb motor.")))
        .withFeedback(
            config.Feedback.withRotorToSensorRatio(POSITION_CONVERSION)
                .withFeedbackRotorOffset(ROTOR_OFFSET)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));
    //
    configurator.refresh(config);

    follower.setControl(new StrictFollower(talon.getDeviceID()));
  }

  @Override
  public void setVoltage(Measure<Voltage> volts) {
    final VoltageOut req = new VoltageOut(0);
    talon.setControl(req.withOutput(volts.in(Volts)));
  }

  @Override
  public Measure<Distance> position() {
    return Meters.of(talon.getPosition().getValueAsDouble() * POSITION_CONVERSION);
  }

  @Override
  public Measure<Velocity<Distance>> velocity() {
    return MetersPerSecond.of(talon.getVelocity().getValueAsDouble() * POSITION_CONVERSION);
  }

  @Override
  public Measure<Voltage> volts() {
    return Volts.of(talon.getMotorVoltage().getValueAsDouble());
  }
}
