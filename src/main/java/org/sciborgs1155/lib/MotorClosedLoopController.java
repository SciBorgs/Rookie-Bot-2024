package org.sciborgs1155.lib;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import monologue.Annotations.Log;
import monologue.Logged;

public class MotorClosedLoopController implements Logged {
  @Log.NT private double goal = 0;
  @Log.NT private double measurement = 0;
  @Log.NT private double initial = 0;
  @Log.NT private double error = 0;

  @Log.NT private double pidOutput = 0;
  @Log.NT private double ffdOutput = 0;
  @Log.NT private double finalOutput = 0;

  private SimpleMotorFeedforward ffdController;
  private ProfiledPIDController pidController;

  public MotorClosedLoopController(
      PIDConstants pidConstants,
      FFDConstants ffdConstants,
      Constraints constraints,
      double positionTolerance,
      double velocityTolerance) {
    pidController =
        new ProfiledPIDController(
            pidConstants.kP, pidConstants.kI, pidConstants.kD, constraints, PERIOD.in(Seconds));
    pidController.setIZone(pidConstants.iZone);
    pidController.setTolerance(positionTolerance, velocityTolerance);

    ffdController = new SimpleMotorFeedforward(ffdConstants.kS, ffdConstants.kV, ffdConstants.kA);
  }

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
  }

  public void setGoal(double goal, double initial) {
    this.initial = initial;
    this.goal = goal;
    this.pidController.setGoal(goal);
  }

  public double getOutput(double measurement) {
    this.measurement = measurement;
    this.pidOutput = pidController.calculate(measurement);
    this.ffdOutput = ffdController.calculate(pidController.getSetpoint().velocity);
    this.finalOutput = (pidOutput + ffdOutput) / 2;
    return finalOutput;
  }

  public boolean isDone() {
    return this.pidController.atGoal();
  }

  public double getError() {
    this.error = pidController.getPositionError();
    return this.error;
  }
}
