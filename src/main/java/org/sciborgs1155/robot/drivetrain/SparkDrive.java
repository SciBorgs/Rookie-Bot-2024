package org.sciborgs1155.robot.drivetrain;

import static org.sciborgs1155.robot.drivetrain.DriveConstants.STARTING_POSE;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
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

  /** We don't have a gyro, so this has to be used with an estimated rotation */
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(STARTING_POSE.getRotation(), 0, 0,
      STARTING_POSE);

  /** Displacements of wheels at last odometry update */
  private final DifferentialDriveWheelPositions previousWheelDisplacements = new DifferentialDriveWheelPositions(0, 0);

  /** Angular velocity of the drivetrain(DegreesPerSecond) */
  private double angularVelocity = 0;

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

  @Override
  @Log.NT
  public double getAngularVelocity() {
    return angularVelocity;
  }

  @Override
  public void updatePose(double deltaTimeSeconds) {
    // The displacement since last odometry update(as opposed to in total)
    double[] deltaDisplacementsMeters = new double[] {
        getLeftDisplacement() - previousWheelDisplacements.leftMeters,
        getRightDisplacement() - previousWheelDisplacements.rightMeters
    };

    // difference in displacement can be used to find a difference in orientation
    double deltaRotationDegrees = distanceToAngle(deltaDisplacementsMeters[1] - deltaDisplacementsMeters[0]);

    // old rotation + delta rotation = new rotation
    double newRotation = getPose().getRotation().getDegrees() + deltaRotationDegrees;

    // Rotation has to be calculated in order to use 'DifferentialDriveOdometry'
    odometry.update(
        Rotation2d.fromDegrees(newRotation), getLeftDisplacement(), getRightDisplacement());

    // angular velocity = delta rotation / delta time
    angularVelocity = deltaRotationDegrees / deltaTimeSeconds;

    // Sets up displacements for next update
    previousWheelDisplacements.leftMeters = getLeftDisplacement();
    previousWheelDisplacements.rightMeters = getRightDisplacement();
  }

  @Override
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

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

    this.frontLeftMotor.burnFlash();
    this.rearLeftMotor.burnFlash();
    this.frontRightMotor.burnFlash();
    this.rearRightMotor.burnFlash();

    this.rearLeftMotor.follow(frontLeftMotor);
    this.rearRightMotor.follow(frontRightMotor);

    FaultLogger.register(frontLeftMotor);
    FaultLogger.register(rearLeftMotor);
    FaultLogger.register(frontRightMotor);
    FaultLogger.register(rearRightMotor);

    FaultLogger.check(frontLeftMotor);
    FaultLogger.check(frontRightMotor);
    FaultLogger.check(rearLeftMotor);
    FaultLogger.check(rearRightMotor);
  }
}
