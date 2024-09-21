// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Elevator extends SubsystemBase implements Logged {

  /* Set up of all components that would be used in the period of 20 ms */
  private ElevatorIO hardware;
  private Measure<Distance> goal = Meters.of(3);
  @Log.NT private double goalMeters = 0;
  @Log.NT private double positionMeters = 0;

  private final TrapezoidProfile profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(5, 5));

  private TrapezoidProfile.State lastRef = new TrapezoidProfile.State();

  /* Setup for the plant, which holds the state-space model */
  // States: [position, velocity] meters, mps
  // Inputs: voltage
  // Outputs: position

  private final LinearSystem<N2, N1, N1> plant =
      LinearSystemId.createElevatorSystem(DCMotor.getCIM(2), 3, Units.inchesToMeters(5), 42);

  /* Set up for observer */
  private final KalmanFilter<N2, N1, N1> observer =
      new KalmanFilter<>(
          Nat.N2(), Nat.N1(), plant, VecBuilder.fill(0.01, 0.01), VecBuilder.fill(0.01), 0.02);

  private final LinearQuadraticRegulator<N2, N1, N1> controller =
      new LinearQuadraticRegulator<>(
          plant,
          VecBuilder.fill(0.0005, 0.001),
          VecBuilder.fill(12.0), // gradually decrease control effort if needed
          0.02);

  // Integrates all previous parts to be used periodically to update the output
  // based on the reference state and the measurement of the current state
  private final LinearSystemLoop<N2, N1, N1> loop =
      new LinearSystemLoop<>(plant, controller, observer, 12.0, 0.02);

  /** Creates a new Elevator. */
  public Elevator(ElevatorIO hardware) {
    this.hardware = hardware;
  }

  public Command moveToDesiredHeight() {
    return run(
        () -> {
          lastRef =
              profile
                  .calculate( // our ref that that has been stepped 0.02 into the predicted future
                      0.02, lastRef, new TrapezoidProfile.State(goal.in(Meters), 0.0));
          loop.setNextR(lastRef.position, lastRef.velocity);

          // Correction of Kalman Filter state vector estimate for sensor (sim) data
          loop.correct(VecBuilder.fill(hardware.currentHeight().in(Meters)));

          loop.predict(0.020);

          Measure<Voltage> output = Volts.of(loop.getU(0));
          hardware.setVoltage(output);

          // update position on a plot
          positionMeters = hardware.currentHeight().in(Meters);
          goalMeters = goal.in(Meters);
        });
  }

  public void setGoal(double height) {
    goal = Meters.of(height);
  }

  public Command setGoal(Measure<Distance> height) {
    return runOnce(() -> setGoal(height.in(Meters)));
  }

  /* Returns a Elevator subystem */
  public static Elevator create() {
    return Robot.isReal() ? new Elevator(new SimElevator()) : new Elevator(new NoElevator());
  }

  // private void setRef(double ref) {

  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
