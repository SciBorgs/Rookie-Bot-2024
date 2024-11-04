package org.sciborgs1155.robot.shooter;

import static org.sciborgs1155.robot.Ports.Wheels.TOP_WHEEL;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.robot.Robot;

public class Shooter extends SubsystemBase {
  private final CANSparkMax motor = new CANSparkMax(TOP_WHEEL, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final PIDController pid = new PIDController(1, 0, 0);
  private final ArmFeedforward ff = new ArmFeedforward(1, 0, 0);
  // where do you get your encoder from?

  private final WheelIO hardware;

  private Shooter(WheelIO hardware) {
    setDefaultCommand(stop());
    this.hardware = hardware;
  }

  /**
   * creates Shooter depending on if it's real or not.
   *
   * @return a shooter with a real wheel if real, else returns shooter with sim wheel.
   */
  public static Shooter create() {
    return Robot.isReal() ? new Shooter(new RealWheel()) : new Shooter(new SimWheel());
  }

  /**
   * creates a null shooter
   *
   * @return a shooter with no wheel.
   */
  public static Shooter none() {
    return new Shooter(new NoWheel());
  }

  // TODO
  // use a PIDController
  // use a SimpleFeedForward
  // in the lambda fed to the shoot Command:
  // get the double output for PID and FF!
  // use {} and put code inside of it in order to write multiple lines in the lambda!
  // give the pid.calculate() method your measurement variable!
  // (how do you get your motor's angular position radians?)
  // give ff.calculate() your velocity setpoint!

  /**
   * turns on the shooter wheels
   *
   * @return command for turning on shooter
   */
  public Command shoot() {
    return run(() -> setSpeed(wheelVelocity));
  }

  /**
   * turns off the shooter wheels
   *
   * @return command for stopping the shooter
   */
  public Command stop() {
    return run(() -> setSpeed(0));
  }

  private void setSpeed(double target) {
    motor.setVoltage(
        pid.calculate(encoder.getVelocity(), target) + ff.calculate(encoder.getVelocity(), target));
  }
}
