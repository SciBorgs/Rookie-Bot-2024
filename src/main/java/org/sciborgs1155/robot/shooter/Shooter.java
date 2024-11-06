package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Ports.Wheels.BOTTOM_WHEEL;
import static org.sciborgs1155.robot.Ports.Wheels.TOP_WHEEL;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import org.sciborgs1155.robot.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Annotations.Log;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import org.sciborgs1155.robot.Robot;

public class Shooter extends SubsystemBase implements Logged {
    private final WheelIO top;
    private final WheelIO bottom;

    private final PIDController pidTop = new PIDController(Top.kp, Top.ki, Top.kd);
    private final PIDController pidBottom = new PIDController(Bottom.kp, Bottom.ki, Bottom.kd);
    private final SimpleMotorFeedforward ffTop = new SimpleMotorFeedforward(Top.ks, Top.kv, Top.ka);
    private final SimpleMotorFeedforward ffBottom = new SimpleMotorFeedforward(Bottom.ks, Bottom.kv, Bottom.ka);

    private Shooter(WheelIO top, WheelIO bottom) {
        setDefaultCommand(stop());
        this.top = top;
        this.bottom = bottom;
    }

    /**
     * creates Shooter depending on if it's real or not.
     *
     * @return a shooter with a real wheel if real, else returns shooter with sim
     *         wheel.
     */
    public static Shooter create() {
        return Robot.isReal() ? new Shooter(new RealWheel(true, TOP_WHEEL), new RealWheel(false, BOTTOM_WHEEL)) : new Shooter(new SimWheel(), new SimWheel());
    }

    /**
     * creates a null shooter
     *
     * @return a shooter with no wheel.
     */
    public static Shooter none() {
        return new Shooter(new NoWheel(), new NoWheel());
    }

    // TODO
    // use a PIDController
    // use a SimpleFeedForward
    // in the lambda fed to the shoot Command:
    // get the double output for PID and FF!
    // use {} and put code inside of it in order to write multiple lines in the
    // lambda!
    // give the pid.calculate() method your measurement variable!
    // (how do you get your motor's angular position radians?)
    // give ff.calculate() your velocity setpoint!

    /**
     * turns on the shooter wheels
     *
     * @return command for turning on shooter
     */
    public Command shoot() {
        return run(() -> setSpeed(wheelVelocity, wheelVelocity));
    }

    /**
     * turns off the shooter wheels
     *
     * @return command for stopping the shooter
     */
    public Command stop() {
        return run(() -> setSpeed(0, 0));
    }

    private void setTopSpeed(double target) {
        double prevTop = pidTop.getSetpoint();
        top.setVoltage(
                pidTop.calculate(top.getVelocityRad(), target) + ffTop.calculate(target, (target-prevTop)/Constants.PERIOD.in(Seconds)));
    }

    private void setBottomSpeed(double target) {
        double prevBottom = pidBottom.getSetpoint();
        top.setVoltage(
                pidBottom.calculate(bottom.getVelocityRad(), target) + ffBottom.calculate(target, (target-prevBottom)/Constants.PERIOD.in(Seconds)));
    }

    private void setSpeed(double topTarget, double topBottom) {
        setTopSpeed(topTarget);
        setBottomSpeed(topBottom);
    }

    @Log.NT
    public double topVelocity() {
        return top.getVelocityRad();
    }

    @Log.NT
    public double bottomVelocity() {
        return bottom.getVelocityRad();
    }    
}
