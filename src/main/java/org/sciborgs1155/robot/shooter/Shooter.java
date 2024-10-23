package org.sciborgs1155.robot.shooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.sciborgs1155.robot.Ports.Wheels.TOP_WHEEL;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import org.sciborgs1155.robot.Robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase{
    private final CANSparkMax motor = new CANSparkMax(TOP_WHEEL, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    private final PIDController pid = new PIDController(1, 0, 0);
    private final ArmFeedforward ff = new ArmFeedforward(1, 0, 0);
    // where do you get your encoder from?

    //TODO
    // use a PIDController
    // use a SimpleFeedForward
    // in the lambda fed to the shoot Command:
        // get the double output for PID and FF!
        // use {} and put code inside of it in order to write multiple lines in the lambda!
        // give the pid.calculate() method your measurement variable! 
        // (how do you get your motor's angular position radians?)
        // give ff.calculate() your velocity setpoint!

    public Command shoot() {
        return run(() -> changeVoltageShoot(wheelVolts));
    }

    private void changeVoltageShoot(double volts) {
        motor.setVoltage(pid.calculate(encoder.getPosition(), volts) + ff.calculate(encoder.getPosition(), volts));
    }

}
