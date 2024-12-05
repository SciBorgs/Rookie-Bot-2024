package org.sciborgs1155.lib;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import monologue.Annotations.Log;
import monologue.Logged;

public class MotorClosedLoopController implements Logged {
  @Log.NT private double goal = 0;
  @Log.NT private double measurement = 0;
  @Log.NT private double initial = 0;
  @Log.NT private double error = 0;

  @Log.NT private double pidOutput = 0;
  @Log.NT private double ffdOutput = 0;

  /** Average of PID and FFD output sent to motors */
  @Log.NT private double finalOutput = 0;

  private SimpleMotorFeedforward ffdController;
  private ProfiledPIDController pidController;

  public MotorClosedLoopController(MotorClosedLoopConstants constants) {
    pidController =
        new ProfiledPIDController(
            constants.KP,
            constants.KI,
            constants.KD,
            new Constraints(constants.MAX_VELOCITY, constants.MAX_ACCELERATION),
            PERIOD.in(Seconds));
    pidController.setIZone(constants.IZONE);
    pidController.setTolerance(constants.POSITION_TOLERANCE, constants.VELOCITY_TOLERANCE);

    ffdController = new SimpleMotorFeedforward(constants.KS, constants.KV, constants.KA);
  }

  /** Re-instantiates the pidController and all fields */
  public void reset() {
    pidController =
        new ProfiledPIDController(
            pidController.getP(),
            pidController.getI(),
            pidController.getD(),
            pidController.getConstraints(),
            pidController.getPeriod());
    pidController.setIZone(pidController.getIZone());
    pidController.setTolerance(
        pidController.getPositionTolerance(), pidController.getVelocityTolerance());

    goal = 0;
    initial = 0;
    error = 0;
    pidOutput = 0;
    ffdOutput = 0;
    finalOutput = 0;
    measurement = 0;
  }

  /** Sets the goal of the pidController and the initial value */
  public void initialize(double goal, double initial) {
    this.initial = initial;
    this.goal = goal;
    this.pidController.setGoal(new State(goal, 0));
  }

  /** Averages PID and FFD outputs and returns(based on measurement and 'setGoal') */
  public double getOutput(double measurement) {
    this.measurement = measurement;
    this.error = goal - measurement;
    this.pidOutput = pidController.calculate(measurement);
    this.ffdOutput = ffdController.calculate(pidController.getSetpoint().velocity);
    this.finalOutput = pidOutput + ffdOutput;
    return finalOutput;
  }

  /** Whether position and velocity are within tolerance */
  public boolean isDone() {
    return this.pidController.atGoal();
  }

  /** Goal - measurement */
  public double getError() {
    return this.error;
  }

  /** Specified in 'setGoal' method */
  public double getInitial() {
    return this.initial;
  }

  public static record MotorClosedLoopConstants(
      double KS,
      double KV,
      double KA,
      double KP,
      double KI,
      double KD,
      double IZONE,
      double POSITION_TOLERANCE,
      double VELOCITY_TOLERANCE,
      double MAX_VELOCITY,
      double MAX_ACCELERATION) {}
}
