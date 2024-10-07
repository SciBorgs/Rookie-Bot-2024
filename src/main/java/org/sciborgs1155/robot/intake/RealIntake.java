package org.sciborgs1155.robot.intake;

import static org.sciborgs1155.robot.Ports.Intake.ROLLER_MOTOR;
import static org.sciborgs1155.robot.Ports.Intake.WRIST_MOTOR;
import static org.sciborgs1155.robot.intake.IntakeConstants.ROLLER_MAX;
import static org.sciborgs1155.robot.intake.IntakeConstants.ROLLER_REVERSE;
import static org.sciborgs1155.robot.intake.IntakeConstants.WRISTD;
import static org.sciborgs1155.robot.intake.IntakeConstants.WRISTI;
import static org.sciborgs1155.robot.intake.IntakeConstants.WRISTP;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;

public class RealIntake implements IntakeIO{

    private final CANSparkMax roller;
    private final CANSparkMax wrist;

    private final PIDController wristController = new PIDController(WRISTP, WRISTI, WRISTD);
    private final AbsoluteEncoder wristEncoder;


    public RealIntake() {
        roller = new CANSparkMax(ROLLER_MOTOR, MotorType.kBrushless);
        wrist = new CANSparkMax(WRIST_MOTOR, MotorType.kBrushless);
        roller.restoreFactoryDefaults();
        roller.burnFlash();
        wrist.restoreFactoryDefaults();
        wrist.burnFlash();
        wristController.reset();
        wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);
    }

    public void roller() {
        roller.set(ROLLER_MAX);
    }

    public void reverse() {
        roller.set(ROLLER_REVERSE);
    }

    public void stop() {
        roller.set(0);
    }

    @Override
    public void setRoller(double speed) {
        roller.set(speed);
    }

    @Override
    public double getPositon() {
        return wristEncoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return wristEncoder.getVelocity();
    }

    @Override
    public void setWrist(double volts) {
        wrist.setVoltage(volts);
    }
}
