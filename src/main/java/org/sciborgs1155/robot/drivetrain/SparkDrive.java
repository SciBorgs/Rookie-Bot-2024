package org.sciborgs1155.robot.drivetrain;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import monologue.Annotations.Log;
import org.sciborgs1155.lib.FaultLogger;

/** {@link DriveIO} class using CANSparkMax controllers */
public class SparkDrive implements DriveIO {
  private CANSparkMax frontLeftMotor;
  private CANSparkMax rearLeftMotor;
  private CANSparkMax frontRightMotor;
  private CANSparkMax rearRightMotor;

  private RelativeEncoder rearLeftEncoder;
  private RelativeEncoder frontLeftEncoder;
  private RelativeEncoder frontRightEncoder;
  private RelativeEncoder rearRightEncoder;

  /**
   * @param motorIDs : [Front Left, Rear Left, Front Right, Rear Right]
   */
  public SparkDrive(int[] motorIDs) {
    this.frontLeftMotor = new CANSparkMax(motorIDs[0], MotorType.kBrushless);
    this.rearLeftMotor = new CANSparkMax(motorIDs[1], MotorType.kBrushless);
    this.frontRightMotor = new CANSparkMax(motorIDs[2], MotorType.kBrushless);
    this.rearRightMotor = new CANSparkMax(motorIDs[3], MotorType.kBrushless);

    this.frontLeftEncoder = frontLeftMotor.getEncoder();
    this.rearLeftEncoder = rearLeftMotor.getEncoder();
    this.frontRightEncoder = frontRightMotor.getEncoder();
    this.rearRightEncoder = rearRightMotor.getEncoder();

    this.frontLeftMotor.restoreFactoryDefaults();
    this.rearLeftMotor.restoreFactoryDefaults();
    this.frontRightMotor.restoreFactoryDefaults();
    this.rearRightMotor.restoreFactoryDefaults();

    this.frontLeftMotor.setIdleMode(IdleMode.kBrake);
    this.rearLeftMotor.setIdleMode(IdleMode.kBrake);
    this.frontRightMotor.setIdleMode(IdleMode.kBrake);
    this.rearRightMotor.setIdleMode(IdleMode.kBrake);

    this.rearLeftMotor.follow(frontLeftMotor);
    this.rearRightMotor.follow(frontRightMotor);

    this.frontLeftMotor.burnFlash();
    this.rearLeftMotor.burnFlash();
    this.frontRightMotor.burnFlash();
    this.rearRightMotor.burnFlash();

    FaultLogger.register(frontLeftMotor);
    FaultLogger.register(rearLeftMotor);
    FaultLogger.register(frontRightMotor);
    FaultLogger.register(rearRightMotor);

    FaultLogger.check(frontLeftMotor);
    FaultLogger.check(frontRightMotor);
    FaultLogger.check(rearLeftMotor);
    FaultLogger.check(rearRightMotor);
  }

  @Override
  public void setLeftVoltage(double volts) {
    // This leads the 'rearRightMotor', so we do not have to set both
    frontLeftMotor.setVoltage(clampVoltage(volts));
  }

  @Override
  @Log.NT
  public double getLeftDisplacement() {
    return ((frontLeftEncoder.getPosition() + rearLeftEncoder.getPosition()) / 2);
  }

  @Override
  @Log.NT
  public double getLeftVelocity() {
    return ((frontLeftEncoder.getVelocity() + rearLeftEncoder.getVelocity()) / 2);
  }

  @Override
  public void resetLeftEncoder() {
    frontLeftEncoder.setPosition(0);
    rearLeftEncoder.setPosition(0);
  }

  @Override
  public void setRightVoltage(double volts) {
    // This leads the 'rearRightMotor', so we do not have to set both
    frontRightMotor.setVoltage(clampVoltage(volts));
  }

  @Override
  @Log.NT
  public double getRightDisplacement() {
    return ((frontRightEncoder.getPosition() + rearRightEncoder.getPosition()) / 2);
  }

  @Override
  @Log.NT
  public double getRightVelocity() {
    return ((frontRightEncoder.getVelocity() + rearRightEncoder.getVelocity()) / 2);
  }

  @Override
  public void resetRightEncoder() {
    frontRightEncoder.setPosition(0);
    rearRightEncoder.setPosition(0);
  }
}
