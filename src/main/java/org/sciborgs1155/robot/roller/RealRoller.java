package org.sciborgs1155.robot.roller;

import static org.sciborgs1155.robot.Ports.Intake.ROLLER_MOTOR;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class RealRoller implements RollerIO {

  private final CANSparkMax roller;

  /** Constructor to create a real roller */
  public RealRoller() {
    roller = new CANSparkMax(ROLLER_MOTOR, MotorType.kBrushless);
    roller.restoreFactoryDefaults();
    roller.setSmartCurrentLimit(30);

    roller.burnFlash();
  }

  /**
   * sets the speed of the roller.
   *
   * @param volts The volts applied to the roller.
   */
  @Override
  public void setRoller(double voltage) {
    roller.setVoltage(voltage);
  }
}
