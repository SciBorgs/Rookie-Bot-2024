package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Constants.isReal;
import static org.sciborgs1155.robot.intake.IntakeConstants.MAX_ACCEL;
import static org.sciborgs1155.robot.intake.IntakeConstants.MAX_VELOCITY;
import static org.sciborgs1155.robot.intake.IntakeConstants.STARTING_ANGLE;
import static org.sciborgs1155.robot.intake.IntakeConstants.WRIST_D;
import static org.sciborgs1155.robot.intake.IntakeConstants.WRIST_DOWN;
import static org.sciborgs1155.robot.intake.IntakeConstants.WRIST_I;
import static org.sciborgs1155.robot.intake.IntakeConstants.WRIST_P;
import static org.sciborgs1155.robot.intake.IntakeConstants.WRIST_UP;
import static org.sciborgs1155.robot.intake.IntakeConstants.kG;
import static org.sciborgs1155.robot.intake.IntakeConstants.kS;
import static org.sciborgs1155.robot.intake.IntakeConstants.kV;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;

public class Intake extends SubsystemBase implements Logged {
  private final IntakeIO hardware;

  @Log.NT
  private final ProfiledPIDController wristController =
      new ProfiledPIDController(
          WRIST_P, WRIST_I, WRIST_D, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCEL));

  private final ArmFeedforward wristFeedforward;

  /**
   * Creates a Intake (Constructor).
   *
   * @param hardware
   */
  private Intake(IntakeIO hardware) {
    this.hardware = hardware;
    wristFeedforward = new ArmFeedforward(kS, kG, kV);
    wristController.setGoal(STARTING_ANGLE.in(Radians));
  }

  /**
   * Creates a real or simulated intake.
   *
   * @return The corresponding created intake.
   */
  public static Intake create() {
    return isReal() ? new Intake(new RealIntake()) : new Intake(new SimIntake());
  }

  /**
   * Creates a no intake.
   *
   * @return The no intake.
   */
  public static Intake none() {
    return new Intake(new NoIntake());
  }

  /**
   * Updates the PID with a setpoint.
   *
   * @param setpoint The setpoint of the wrist in radians.
   */
  private void updatePosition(double setpoint) {
    double PIDOutput = wristController.calculate(hardware.getPosition(), setpoint);
    double FFOutput = wristFeedforward.calculate(hardware.getPosition(), hardware.getVelocity());
    hardware.setWristVoltage(PIDOutput + FFOutput);
  }

  @Log.NT
  public double position() {
    return hardware.getPosition();
  }

  @Log.NT
  public double goal() {
    return wristController.getGoal().position;
  }

  @Log.NT
  public double setpoint() {
    return wristController.getSetpoint().position;
  }

  /**
   * A command to raise the wrist.
   *
   * @return A command to raise the wrist.
   */
  public Command raiseWrist() {
    return run(() -> updatePosition(WRIST_UP));
  }

  /**
   * A command to lower the wrist.
   *
   * @return A command to lower the wrist.
   */
  public Command lowerWrist() {
    return run(() -> updatePosition(WRIST_DOWN));
  }
}
