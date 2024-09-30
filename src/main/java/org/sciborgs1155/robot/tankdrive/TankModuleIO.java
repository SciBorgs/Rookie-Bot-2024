package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import monologue.Logged;

/** Generalized hardware internals for a swerve module */
public interface TankModuleIO extends AutoCloseable, Logged, Subsystem {
  /**
   * Sets the drive voltage of the module.
   *
   * @param voltage : The voltage to inputted into the drive motor.
   */
  public Command setVoltage(Measure<Voltage> voltage);

  /**
   * Sets the drive voltage of the module.
   *
   * @param voltage : Voltage in volts.
   */
  default void setVoltage(double volts) {
    CommandScheduler.getInstance().schedule(setVoltage(Volts.of(volts)));
  }

  /**
   * Returns simulated instance of one wheel of a tank module.
   *
   * @return sim module.
   */
  public DCMotorSim getMotorSim();

  /**
   * Returns the distance the wheel traveled.
   *
   * @return The drive encoder position value, in radians.
   */
  public Measure<Distance> getPosition();

  /**
   * Returns the current velocity of the wheel.
   *
   * @return The drive encoder velocity value, in radians / seconds.
   */
  public Measure<Velocity<Distance>> getVelocity();

  /** Resets all encoders. */
  public void resetEncoders();

  /** Returns the name of this module(Format: Type-XX) */
  public String getName();

  @Override
  public void close() throws Exception;

  public class NoModule implements TankModuleIO {
    @Override
    public Command setVoltage(Measure<Voltage> voltage) {
      return Commands.idle(this);
    }

    @Override
    public Measure<Distance> getPosition() {
      return Meters.of(0);
    }

    @Override
    public Measure<Velocity<Distance>> getVelocity() {
      return MetersPerSecond.of(0);
    }

    @Override
    public void resetEncoders() {}

    @Override
    public String getName() {
      return "Non-existent Module";
    }

    @Override
    public DCMotorSim getMotorSim() {
      return null;
    }

    @Override
    public void close() throws Exception {
      this.close();
    }
  }
}
