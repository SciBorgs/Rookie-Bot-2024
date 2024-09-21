// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.climber;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.climber.ClimbConstants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Climber extends SubsystemBase implements Logged {

  private ClimberIO hardware;

  @Log.NT
  private ProfiledPIDController pid =
      new ProfiledPIDController(kP, kI, kD, new Constraints(MAX_VELO, MAX_ACCEL));

  @Log.NT private ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV, kA);

  @Log.NT public double position = 0;
  private SysIdRoutine routine;
  private SysIdRoutineLog log;

  /** Creates a new Climber. */
  public Climber(ClimberIO hardware) {
    this.hardware = hardware;

    pid.setTolerance(1E-5);

    log = new SysIdRoutineLog("Climber! :D");
    routine =
        new SysIdRoutine(
            new Config(),
            new Mechanism(
                (volts) -> hardware.setVoltage(volts),
                log -> {
                  log.motor("Climb motor")
                      .linearPosition(hardware.position())
                      .linearVelocity(hardware.velocity())
                      .voltage(hardware.volts());
                },
                this,
                "Climb"));
    SmartDashboard.putData("climber quasistatic-forward", climberQuasistatic(Direction.kForward));
    SmartDashboard.putData("climber quasistatic-back", climberQuasistatic(Direction.kReverse));
    SmartDashboard.putData("climber dynamic-forward", climberDynamic(Direction.kForward));
    SmartDashboard.putData("climber dynamic-back", climberDynamic(Direction.kReverse));

    SmartDashboard.putData("climber-sysid test", runClimberSysid());
  }

  public static Climber create() {
    return Robot.isReal() ? new Climber(new RealClimber()) : new Climber(new SimClimber());
  }

  public static Climber none() {
    return new Climber(new NoClimber());
  }

  private Command moveToGoal() {
    return run(
        () -> {
          double pidOutput = pid.calculate(hardware.position().in(Meters));
          double ffOutput = ff.calculate(pid.getSetpoint().velocity, 0);

          hardware.setVoltage(Volts.of(pidOutput + ffOutput));
          position = hardware.position().in(Meters);
        });
  }

  private Command setGoal(Measure<Distance> height) {
    return runOnce(() -> pid.setGoal(height.in(Meters)));
  }

  // the actually accessible command to be used when running climb
  public Command moveToGoal(Measure<Distance> height) {
    return setGoal(height).andThen(moveToGoal());
  }

  public Command climberQuasistatic(Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command climberDynamic(Direction direction) {
    return routine.dynamic(direction);
  }

  public Command runClimberSysid() {
    return climberQuasistatic(Direction.kForward)
        .andThen(climberQuasistatic(Direction.kReverse))
        .andThen(climberDynamic(Direction.kForward))
        .andThen(climberDynamic(Direction.kReverse))
        .andThen(runOnce(() -> SmartDashboard.putBoolean("Climb SYSID Completed", true)))
        .withName("Running Climb SysID");
  }

  public Measure<Distance> height() {
    return hardware.position();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
