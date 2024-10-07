package org.sciborgs1155.robot.intake;

import static org.sciborgs1155.robot.Constants.isReal;
import static org.sciborgs1155.robot.intake.IntakeConstants.ROLLER_MAX;
import static org.sciborgs1155.robot.intake.IntakeConstants.ROLLER_REVERSE;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private IntakeIO hardware;

  private Intake(IntakeIO hardware) {
    this.hardware = hardware;
  }

  public static Intake create() {
    return isReal() ? new Intake(new RealIntake()) : new Intake(new SimIntake());
  }

  public static Intake none() {
    return new Intake(new NoIntake());
  }

  private void setRoller(double speed) {
    hardware.setRoller(speed);
  }

  public Command roller() {
    return run(() -> setRoller(ROLLER_MAX));
  }

  public Command stopRoller() {
    return run(() -> setRoller(0));
  }

  public Command reverseRoller() {
    return run(() -> setRoller(ROLLER_REVERSE));
  }
}
