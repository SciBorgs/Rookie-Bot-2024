package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.STARTING_POSE;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.clampVoltage;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.distanceToAngle;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import org.sciborgs1155.lib.FaultLogger;

/** {@link DiffDriveIO} class using CANSparkMax controllers. */
public class SparkDiffDrive implements DiffDriveIO {
  private CANSparkMax frontLeftMotor;
  private CANSparkMax rearLeftMotor;
  private CANSparkMax frontRightMotor;
  private CANSparkMax rearRightMotor;

  private RelativeEncoder rearLeftEncoder;
  private RelativeEncoder frontLeftEncoder;
  private RelativeEncoder frontRightEncoder;
  private RelativeEncoder rearRightEncoder;

  /**
   * Displacements of motors, and rotation of the robot at last odometry update (1: Left
   * Displacement, 2: Right Displacement, 3: Rotation, 4: Angular Velocity). Used for calculating
   * robot rotation / angular velocity. (since we don't have a gyro :/ ) Angular velocity is simply
   * being recorded for 'getAngularVelocity' method
   */
  private double[] previousDisplacements = new double[] {0.0, 0.0, 0.0, 0.0};

  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(STARTING_POSE.getRotation(), 0, 0, STARTING_POSE);

  @Override
  public Measure<Voltage> setLeftVoltage(Measure<Voltage> voltage) {
    frontRightMotor.setVoltage(clampVoltage(voltage).in(Volts));
    rearRightMotor.setVoltage(clampVoltage(voltage).in(Volts));

    return voltage;
  }

  /**
   * Total displacement of the left side of the drivetrain (averages the velocities of both motors
   * for robustness): i.e. both motors going opposite directions
   */
  @Override
  public Measure<Distance> getLeftDisplacement() {
    return Meters.of((frontLeftEncoder.getPosition() + rearLeftEncoder.getPosition()) / 2);
  }

  /**
   * The current velocity of the left side of the drivetrain (averages the velocities of both motors
   * for robustness): i.e. both motors going opposite directions
   */
  @Override
  public Measure<Velocity<Distance>> getLeftVelocity() {
    return MetersPerSecond.of((frontLeftEncoder.getVelocity() + rearLeftEncoder.getVelocity()) / 2);
  }

  @Override
  public void resetLeftEncoder() {
    frontLeftEncoder.setPosition(0);
    rearLeftEncoder.setPosition(0);
  }

  @Override
  public Measure<Voltage> setRightVoltage(Measure<Voltage> voltage) {
    frontRightMotor.setVoltage(clampVoltage(voltage).in(Volts));
    rearRightMotor.setVoltage(clampVoltage(voltage).in(Volts));

    return voltage;
  }

  /**
   * Total displacement of the right side of the drivetrain (averages the velocities of both motors
   * for robustness): i.e. both motors going opposite directions
   */
  @Override
  public Measure<Distance> getRightDisplacement() {
    return Meters.of((frontRightEncoder.getPosition() + rearRightEncoder.getPosition()) / 2);
  }

  /**
   * The current velocity of the right side of the drivetrain (averages the velocities of both
   * motors for robustness): i.e. both motors going opposite directions
   */
  @Override
  public Measure<Velocity<Distance>> getRightVelocity() {
    return MetersPerSecond.of(
        (frontRightEncoder.getVelocity() + rearRightEncoder.getVelocity()) / 2);
  }

  @Override
  public Measure<Velocity<Angle>> getAngularVelocity() {
    return DegreesPerSecond.of(previousDisplacements[3]);
  }

  @Override
  public void resetRightEncoder() {
    frontRightEncoder.setPosition(0);
    rearRightEncoder.setPosition(0);
  }

  /**
   * @param motorIDs : [Front Left, Rear Left, Front Right, Rear Right]
   */
  public SparkDiffDrive(int[] motorIDs) {
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

    FaultLogger.register(frontLeftMotor);
    FaultLogger.register(rearLeftMotor);
    FaultLogger.register(frontRightMotor);
    FaultLogger.register(rearRightMotor);
  }

  @Override
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  @Override
  public Pose2d updatePose(Measure<Time> deltaTime) {
    // For calculating robot rotation
    double[] deltaDisplacements =
        new double[] {
          getLeftDisplacement().in(Meters) - previousDisplacements[0],
          getRightDisplacement().in(Meters) - previousDisplacements[1]
        };
    Rotation2d deltaRotation =
        new Rotation2d(distanceToAngle(Meters.of(deltaDisplacements[1] - deltaDisplacements[0])));

    Rotation2d newRotation = getPose().getRotation().plus(deltaRotation);
    odometry.update(
        newRotation, getLeftDisplacement().in(Meters), getRightDisplacement().in(Meters));

    Measure<Velocity<Angle>> angularVelocity =
        DegreesPerSecond.of(deltaRotation.getDegrees() / deltaTime.in(Seconds));

    previousDisplacements[0] = getLeftDisplacement().in(Meters);
    previousDisplacements[1] = getRightDisplacement().in(Meters);
    previousDisplacements[2] = newRotation.getDegrees();
    previousDisplacements[3] = angularVelocity.in(DegreesPerSecond);

    return getPose();
  }
}
