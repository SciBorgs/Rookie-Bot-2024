package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.robot.Constants;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Represents two SparkMaxes on one side of a differential(tank) drivetrain. */
public class SparkModule implements TankModuleIO {
  /** Front motor. */
  private CANSparkMax frontMotor;

  /** Rear motor. */
  private CANSparkMax rearMotor;

  /** Front encoder. */
  private RelativeEncoder frontEncoder;

  /** Rear encoder. */
  private RelativeEncoder rearEncoder;

  @Override
  public Command setVoltage(Measure<Voltage> voltage) {
    return Commands.runOnce(
        () -> {
          // Updates input voltages.
          frontMotor.setVoltage(voltage.in(Volts));
          rearMotor.setVoltage(voltage.in(Volts));
        },
        this).withName("setVoltage(" + voltage.in(Volts) + ")").andThen(Commands.idle(this));
  }

  @Override
  public Measure<Distance> getDisplacement() {
    // Averages displacements of both motors.
    return Meters.of((frontEncoder.getPosition() + rearEncoder.getPosition()) / 2);
  }

  @Override
  public Measure<Velocity<Distance>> getVelocity() {
    // Averages velocities of both motors.
    return MetersPerSecond.of((frontEncoder.getVelocity() + rearEncoder.getVelocity()) / 2);
  }

  @Override
  public void resetEncoders() {
    // Resets displacement measurements.
    frontEncoder.setPosition(0);
    rearEncoder.setPosition(0);
  }

  /** Creates an instance of a Sim Module. */
  public static TankModuleIO create(int frontMotorID, int rearMotorID) {
    return new SparkModule(frontMotorID, rearMotorID);
  }

  private SparkModule(int frontMotorID, int rearMotorID) {
    // Instantiates motors and encoders.
    this.frontMotor = new CANSparkMax(frontMotorID, MotorType.kBrushless);
    this.frontEncoder = frontMotor.getEncoder();

    this.rearMotor = new CANSparkMax(rearMotorID, MotorType.kBrushless);
    this.rearEncoder = rearMotor.getEncoder();

    // Encoders update every 20ms.
    this.frontEncoder.setMeasurementPeriod((int) Constants.PERIOD.in(Seconds));
    this.rearEncoder.setMeasurementPeriod((int) Constants.PERIOD.in(Seconds));

    // Converts rotations and rotations per second into meters and meters/second.
    this.frontEncoder.setVelocityConversionFactor(DriveConstants.WHEEL_RADIUS.in(Meters));
    this.frontEncoder.setVelocityConversionFactor(DriveConstants.WHEEL_RADIUS.in(Meters));

    this.frontEncoder.setPositionConversionFactor(DriveConstants.WHEEL_RADIUS.in(Meters));
    this.frontEncoder.setPositionConversionFactor(DriveConstants.WHEEL_RADIUS.in(Meters));

    // Logs faults.
    FaultLogger.register(frontMotor);
    FaultLogger.register(rearMotor);
  }

  @Override
  public void close() throws Exception {
    this.close();
  }
}
