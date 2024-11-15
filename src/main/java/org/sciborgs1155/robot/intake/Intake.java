package org.sciborgs1155.robot.intake;

import static org.sciborgs1155.robot.Constants.isReal;
import static org.sciborgs1155.robot.intake.IntakeConstants.MAX_ACCEL;
import static org.sciborgs1155.robot.intake.IntakeConstants.MAX_VELOCITY;
import static org.sciborgs1155.robot.intake.IntakeConstants.WRIST_D;
import static org.sciborgs1155.robot.intake.IntakeConstants.WRIST_DOWN;
import static org.sciborgs1155.robot.intake.IntakeConstants.WRIST_I;
import static org.sciborgs1155.robot.intake.IntakeConstants.WRIST_P;
import static org.sciborgs1155.robot.intake.IntakeConstants.WRIST_UP;
import static org.sciborgs1155.robot.intake.IntakeConstants.kA;
import static org.sciborgs1155.robot.intake.IntakeConstants.kS;
import static org.sciborgs1155.robot.intake.IntakeConstants.kV;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final IntakeIO hardware;

    private final ProfiledPIDController wristController = new ProfiledPIDController(WRIST_P, WRIST_I, WRIST_D, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCEL));
    private final ArmFeedforward wristFeedforward;


    /**
     * Creates a Intake (Constructor).
     * @param hardware
     */
    private Intake(IntakeIO hardware) {
        this.hardware = hardware;
        wristFeedforward = new ArmFeedforward(kS, kV, kA);
    }

    /**
     * Creates a real or simulated intake.
     * @return The corresponding created intake.
     */
    public static Intake create() {
        return isReal() ? new Intake(new RealIntake()) : new Intake(new SimIntake());
    }

    /**
     * Creates a no intake.
     * @return The no intake.
     */
    public static Intake none() {
        return new Intake(new NoIntake());
    }

    /**
     * Updates the PID with a setpoint.
     * @param setpoint
     */
    private void updatePosition(double setpoint) {
        double PIDOutput = wristController.calculate(hardware.getPosition(), setpoint);
        double FFOutput = wristFeedforward.calculate(wristController.getSetpoint().position, wristController.getSetpoint().velocity);
        hardware.setWristVoltage(PIDOutput + FFOutput);
    }

    /**
     * A command to raise the wrist.
     * @return A command to raise the wrist.
     */
    public Command raiseWrist() {
        return run(() -> updatePosition(WRIST_UP));
    }

    /**
     * A command to lower the wrist.
     * @return A command to lower the wrist.
     */
    public Command lowerWrist() {
        return run(() -> updatePosition(WRIST_DOWN));
    }
}
