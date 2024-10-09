package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import monologue.Annotations.Log;

public class SimModule implements TankModuleIO {
  /** Name. */
  private String name = "SimModule";

  /** Last time that the sim was update. */
  private Measure<Time> lastTime = Seconds.of(Timer.getFPGATimestamp());

  @Log.NT
  /** Simulated motors. */
  private final DCMotorSim motors = new DCMotorSim(DCMotor.getNEO(2), DriveConstants.REDUCTION,
      DriveConstants.MOI_MASS.in(Kilograms));

  @Override
  public Command setVoltage(Measure<Voltage> voltage) {
    return Commands.runOnce(
        () -> {
          // Updates input voltages of the motors.
          motors.setInputVoltage(voltage.in(Volts));
          motors.update(Timer.getFPGATimestamp() - lastTime.in(Seconds));
          lastTime = Seconds.of(Timer.getFPGATimestamp());
        },
        this)
        .andThen(Commands.idle(this)).withName("setVoltage(" + voltage.in(Volts) + ")");
  }

  @Override
  public Measure<Distance> getPosition() {
    return Meters.of(
        motors.getAngularPositionRotations()
            * DriveConstants.WHEEL_RADIUS.in(Meters)
            * Math.PI);
  }

  @Override
  public Measure<Velocity<Distance>> getVelocity() {
    return MetersPerSecond.of(
        motors.getAngularVelocityRPM()
            * DriveConstants.WHEEL_RADIUS.in(Meters)
            * Math.PI);
  }

  /** NOTE: This resets the module positions. */
  @Override
  public void resetEncoders() {
    motors.setState(0, 0);
  }

  @Override
  public String getName() {
    return this.name;
  }

  /** Creates an instance of a Sim Module. */
  public static TankModuleIO create(String name) {
    return new SimModule(name);
  }

  private SimModule(String name) {
    this.name = "SimModule: " + name;
  }

  @Override
  public void close() {
    this.close();
  }
}
