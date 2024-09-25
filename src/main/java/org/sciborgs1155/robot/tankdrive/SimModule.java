package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.sciborgs1155.robot.tankdrive.DriveConstants.SimConstants;

public class SimModule implements TankModuleIO {
  /** Name. */
  private String name = "SimModule";

  /** Front Motor Simulation. */
  private final DCMotorSim frontMotor = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(SimConstants.VELOCITY_GAIN, SimConstants.ACCELERATION),
      DCMotor.getNeoVortex(1),
      SimConstants.GEARING);

  /** Rear Motor Simulation. */
  private final DCMotorSim rearMotor = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(SimConstants.VELOCITY_GAIN, SimConstants.ACCELERATION),
      DCMotor.getNeoVortex(1),
      SimConstants.GEARING);

  @Override
  public Command setVoltage(Measure<Voltage> voltage) {
    return Commands.runOnce(
        () -> {
          frontMotor.setInputVoltage(voltage.in(Volts));
          rearMotor.setInputVoltage(voltage.in(Volts));
        },
        this)
        .andThen(Commands.idle(this));
  }

  @Override
  public Measure<Distance> getPosition() {
    return Meters.of(
        (frontMotor.getAngularPositionRotations() + rearMotor.getAngularPositionRotations())
            * DriveConstants.WHEEL_RADIUS.in(Meters)
            * Math.PI);
  }

  @Override
  public Measure<Velocity<Distance>> getVelocity() {
    return MetersPerSecond.of(
        (frontMotor.getAngularVelocityRPM() + rearMotor.getAngularVelocityRPM())
            * DriveConstants.WHEEL_RADIUS.in(Meters)
            * Math.PI);
  }

  /** NOTE: This resets the module positions. */
  @Override
  public void resetEncoders() {
    frontMotor.setState(0, 0);
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

  @Override
  public DCMotorSim getMotorSim() {
    return new DCMotorSim(
        LinearSystemId.createDCMotorSystem(SimConstants.VELOCITY_GAIN, SimConstants.ACCELERATION),
        DCMotor.getNeoVortex(1),
        SimConstants.GEARING);
  }
}
