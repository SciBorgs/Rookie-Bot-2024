package org.sciborgs1155.robot.intake;

import static org.sciborgs1155.robot.Ports.Intake.WRIST_MOTOR;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

public class RealIntake implements IntakeIO {

  private final CANSparkMax wrist;

  private final AbsoluteEncoder wristEncoder;

  /** Real Intake constructor. */
  public RealIntake() {
    wrist = new CANSparkMax(WRIST_MOTOR, MotorType.kBrushless);

    wrist.restoreFactoryDefaults();
    wrist.burnFlash();
    wrist.setIdleMode(IdleMode.kBrake);

    wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);
  }

  /**
   * Gets the position of the wrist.
   *
   * @return The position of the wrist.
   */
  @Override
  public double getPosition() {
    return wristEncoder.getPosition();
  }

  /**
   * Gets the velocity of the wrist
   *
   * @return The velocity of the wrist.
   */
  @Override
  public double getVelocity() {
    return wristEncoder.getVelocity();
  }

  /**
   * Set the voltage of the wrist.
   *
   * @param volts The voltage applied to the wrist.
   */
  @Override
  public void setWristVoltage(double volts) {
    wrist.setVoltage(volts);
  }
}
