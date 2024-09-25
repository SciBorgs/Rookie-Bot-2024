package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.robot.Constants;

public class SparkModule implements TankModuleIO {
  /** Front motor. */
  private CANSparkMax frontMotor;

  /** Rear motor. */
  private CANSparkMax rearMotor;

  /** Front encoder. */
  private RelativeEncoder frontEncoder;

  /** Rear encoder. */
  private RelativeEncoder rearEncoder;

  /** Name. */
  private String name;

  @Override
  public Command setVoltage(Measure<Voltage> voltage) {
    return Commands.runOnce(
            () -> {
              frontMotor.setVoltage(voltage.in(Volts));
              rearMotor.setVoltage(voltage.in(Volts));
            },
            this)
        .andThen(Commands.idle(this));
  }

  @Override
  public Measure<Distance> getPosition() {
    return Meters.of((frontEncoder.getPosition() + rearEncoder.getPosition()) / 2);
  }

  @Override
  public Measure<Velocity<Distance>> getVelocity() {
    return MetersPerSecond.of((frontEncoder.getVelocity() + rearEncoder.getVelocity()) / 2);
  }

  @Override
  public void resetEncoders() {
    frontEncoder.setPosition(0);
    rearEncoder.setPosition(0);
  }

  @Override
  public String getName() {
    return name;
  }

  /** Creates an instance of a Sim Module. */
  public static TankModuleIO create(int frontMotorID, int rearMotorID, String name) {
    return new SparkModule(frontMotorID, rearMotorID, name);
  }

  private SparkModule(int frontMotorID, int rearMotorID, String name) {
    // Instantiates motors and encoders.
    this.frontMotor = new CANSparkMax(frontMotorID, MotorType.kBrushless);
    this.frontEncoder = frontMotor.getEncoder();

    this.rearMotor = new CANSparkMax(rearMotorID, MotorType.kBrushless);
    this.rearEncoder = rearMotor.getEncoder();

    // Encoders update every 20ms.
    this.frontEncoder.setMeasurementPeriod((int) Constants.PERIOD.in(Seconds));
    this.rearEncoder.setMeasurementPeriod((int) Constants.PERIOD.in(Seconds));

    // Converts rotations and rotations per second into meters and meters per
    // second.
    this.frontEncoder.setVelocityConversionFactor(DriveConstants.WHEEL_RADIUS.in(Meters));
    this.frontEncoder.setVelocityConversionFactor(DriveConstants.WHEEL_RADIUS.in(Meters));

    this.frontEncoder.setPositionConversionFactor(DriveConstants.WHEEL_RADIUS.in(Meters));
    this.frontEncoder.setPositionConversionFactor(DriveConstants.WHEEL_RADIUS.in(Meters));

    // Instantiates name.
    this.name = name;

    // Logs faults.
    FaultLogger.register(frontMotor);
    FaultLogger.register(rearMotor);
  }

  @Override
  public void close() throws Exception {
    this.close();
  }
}
