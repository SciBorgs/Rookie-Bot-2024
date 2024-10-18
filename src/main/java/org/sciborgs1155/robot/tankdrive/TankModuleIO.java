package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import monologue.Logged;

/** Represents two motors on one side of a differential(tank) drivetrain. */
public interface TankModuleIO extends AutoCloseable, Logged, Subsystem {
  /**
   * Sets the voltage of both of the motors in the module.
   * 
   * @param voltage : Voltage.
   */
  public Command setVoltage(Measure<Voltage> voltage);

  /**
   * Sets the voltage of both of the motors in the module.
   *
   * @param volts : Voltage(volts).
   */
  default void setVoltage(double volts) {
    CommandScheduler.getInstance().schedule(setVoltage(Volts.of(volts)));
  }

  /**
   * Returns the net displacement that the module has traveled.
   *
   * @return Displacement.
   */
  public Measure<Distance> getDisplacement();

  /**
   * Returns the net displacement that the module has traveled.
   *
   * @return Displacement(Meters).
   */
  default double getDisplacementDouble() {
    return getDisplacement().in(Meters);
  }

  /**
   * Returns the current velocity of the module.
   *
   * @return Velocity.
   */
  public Measure<Velocity<Distance>> getVelocity();

  /**
   * Returns the current velocity of the module.
   *
   * @return Velocity(Meters per second).
   */
  default double getVelocityDouble() {
    return getVelocity().in(MetersPerSecond);
  }

  /** Resets displacement measurement. */
  public void resetEncoders();

  @Override
  public void close() throws Exception;

  public class NoModule implements TankModuleIO {
    @Override
    public Command setVoltage(Measure<Voltage> voltage) {
      return Commands.idle(this);
    }

    @Override
    public Measure<Distance> getDisplacement() {
      return Meters.of(0);
    }

    @Override
    public Measure<Velocity<Distance>> getVelocity() {
      return MetersPerSecond.of(0);
    }

    @Override
    public void resetEncoders() {
    }

    public static TankModuleIO create() {
      return new NoModule();
    }

    @Override
    public void close() throws Exception {
      this.close();
    }
  }
}
