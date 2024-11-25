package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Ports.Wheels.BOTTOM_WHEEL;
import static org.sciborgs1155.robot.Ports.Wheels.TOP_WHEEL;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;

public class Shooter extends SubsystemBase implements Logged {
  private final WheelIO top;
  private final WheelIO bottom;

  private final PIDController pidTop = new PIDController(Top.kp, Top.ki, Top.kd);
  private final PIDController pidBottom = new PIDController(Bottom.kp, Bottom.ki, Bottom.kd);
  private final SimpleMotorFeedforward ffTop = new SimpleMotorFeedforward(Top.ks, Top.kv, Top.ka);
  private final SimpleMotorFeedforward ffBottom =
      new SimpleMotorFeedforward(Bottom.ks, Bottom.kv, Bottom.ka);

  private Shooter(WheelIO top, WheelIO bottom) {
    setDefaultCommand(stop());
    this.top = top;
    this.bottom = bottom;
  }

  /**
   * Creates a real wheel if the robot is real, and a simulated wheel if the robot is not real.
   *
   * @return a shooter with a real wheel if real, else returns shooter with sim wheel.
   */
  public static Shooter create() {
    return Robot.isReal()
        ? new Shooter(new RealWheel(true, TOP_WHEEL), new RealWheel(false, BOTTOM_WHEEL))
        : new Shooter(new SimWheel(), new SimWheel());
  }

  /**
   * Creates a non-existant shooter.
   *
   * @return a shooter with no wheel.
   */
  public static Shooter none() {
    return new Shooter(new NoWheel(), new NoWheel());
  }

  /**
   * Turns on the shooter wheels.
   *
   * @return a command for turning on the shooter.
   */
  public Command shoot() {
    return setSpeed(WHEEL_VELOCITY);
  }

  /**
   * Turns off the shooter wheels.
   *
   * @return a command for stopping the shooter.
   */
  public Command stop() {
    return setSpeed(0);
  }

  /**
   * Sets the speed of the top wheel.
   *
   * @param target The target speed of the top wheel.
   */
  private void setTopSpeed(double target) {
    double prevTop = pidTop.getSetpoint();
    top.setVoltage(
        pidTop.calculate(top.getVelocity(), target)
            + ffTop.calculate(target, (target - prevTop) / Constants.PERIOD.in(Seconds)));
  }

  /**
   * Sets the speed of the bottom wheel.
   *
   * @param target The target speed of the bottom wheel.
   */
  private void setBottomSpeed(double target) {
    double prevBottom = pidBottom.getSetpoint();
    bottom.setVoltage(
        pidBottom.calculate(bottom.getVelocity(), target)
            + ffBottom.calculate(target, (target - prevBottom) / Constants.PERIOD.in(Seconds)));
  }

  /**
   * Sets speed of both top and bottom wheels.
   *
   * @param Target The target speed of the wheels.
   */
  private Command setSpeed(double topTarget, double bottomTarget) {
    return run(
        () -> {
          setTopSpeed(topTarget);
          setBottomSpeed(bottomTarget);
        });
  }

  private Command setSpeed(double target) {
    return setSpeed(target, target);
  }

  /**
   * Returns the top wheel's velocity.
   *
   * @return the top wheel's velocity.
   */
  @Log.NT
  public double topVelocity() {
    return top.getVelocity();
  }

  /**
   * Returns the bottom wheel's velocity.
   *
   * @return the bottom wheel's velocity.
   */
  @Log.NT
  public double bottomVelocity() {
    return bottom.getVelocity();
  }

  @Log.NT
  public double topTarget() {
    return pidTop.getSetpoint();
  }

  @Log.NT
  public double bottomTarget() {
    return pidBottom.getSetpoint();
  }
}
