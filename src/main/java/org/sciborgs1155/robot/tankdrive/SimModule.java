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

/** A simulation of one side of a differential(tank) drivetrain. */
public class SimModule implements TankModuleIO {
  /** Last FPGA timestamp that the sim had been updated. */
  private Measure<Time> lastTime;

  /** Simulated instance of one side of the drivetrain(2 neos). */
  private DCMotorSim motors = new DCMotorSim(DCMotor.getNEO(2),
      DriveConstants.REDUCTION,
      DriveConstants.MOI_MASS.in(Kilograms));

  @Override
  public Command setVoltage(Measure<Voltage> voltage) {
    return Commands.runOnce(
        () -> {
          // Updates input voltages of the motors.
          motors.setInputVoltage(voltage.in(Volts));

          // Updates the simulation time of the simulated motors.
          motors.update(Timer.getFPGATimestamp() - lastTime.in(Seconds));
          lastTime = Seconds.of(Timer.getFPGATimestamp());
        },
        this).withName("setVoltage(" + voltage.in(Volts) + ")").andThen(Commands.idle(this));
  }

  @Override
  public Measure<Distance> getDisplacement() {
    // Converts angular displacement to linear displacement.
    return Meters.of(
        motors.getAngularPositionRotations()
            * DriveConstants.WHEEL_RADIUS.in(Meters)
            * Math.PI * 2);
  }

  @Override
  public Measure<Velocity<Distance>> getVelocity() {
    // Converts motor angular velocity into linear velocity.
    return MetersPerSecond.of(
        motors.getAngularVelocityRPM()
            * DriveConstants.WHEEL_RADIUS.in(Meters)
            * Math.PI * 2);
  }

  /** Creates an instance of a Sim Module. */
  public static TankModuleIO create() {
    return new SimModule();
  }

  /** Creates an instance of a Sim Module. */
  private SimModule() {
    // Instantiation.
    motors = new DCMotorSim(DCMotor.getNEO(2), DriveConstants.REDUCTION, DriveConstants.MOI_MASS.in(Kilograms));
    lastTime = Seconds.of(Timer.getFPGATimestamp());
  }

  /** NOTE: This resets the module rotations without offsetting the readings. */
  @Override
  public void resetEncoders() {
    motors.setState(0, 0);
  }

  @Override
  public void close() {
    this.close();
  }
}
