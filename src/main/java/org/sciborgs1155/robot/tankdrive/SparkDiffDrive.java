package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.STARTING_POSE;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.distanceToAngle;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;

/** {@link DiffDriveIO} class using CANSparkMax controllers. */
public class SparkDiffDrive implements DiffDriveIO {
  /** Front left motor. */
  private CANSparkMax frontLeftMotor;

  /** Front left encoder. */
  private RelativeEncoder frontLeftEncoder;

  /** Rear left motor. */
  private CANSparkMax rearLeftMotor;

  /** Rear left encoder. */
  private RelativeEncoder rearLeftEncoder;

  /** Front right motor. */
  private CANSparkMax frontRightMotor;

  /** Front right encoder. */
  private RelativeEncoder frontRightEncoder;

  /** Rear right motor. */
  private CANSparkMax rearRightMotor;

  /** Rear right encoder. */
  private RelativeEncoder rearRightEncoder;

  /** Estimated Position(in Meters). */
  private Pose2d pose;

  /**
   * Displacements at last estimated pose
   * update(left(Meters),right(Meters),angle(degrees)).
   */
  private double[] prevDisplacements;

  /** Rotation at last estimated pose update(Degrees). */
  private double prevRotation;

  @Override
  public Command setLeftVoltage(Measure<Voltage> voltage) {
    return runOnce(
        () -> {
          // Updates input voltages.
          frontLeftMotor.setVoltage(voltage.in(Volts));
          rearLeftMotor.setVoltage(voltage.in(Volts));
        })
        .withName("setLeftVoltage(" + voltage.in(Volts) + ")");
  }

  @Override
  public Measure<Distance> getLeftDisplacement() {
    // Averages displacements of both motors.
    return Meters.of((frontLeftEncoder.getPosition() + rearLeftEncoder.getPosition()) / 2);
  }

  @Override
  public Measure<Velocity<Distance>> getLeftVelocity() {
    // Averages velocities of both motors.
    return MetersPerSecond.of((frontLeftEncoder.getVelocity() + rearLeftEncoder.getVelocity()) / 2);
  }

  @Override
  public void resetLeftEncoder() {
    // Resets displacement measurements.
    frontLeftEncoder.setPosition(0);
    rearLeftEncoder.setPosition(0);
  }

  @Override
  public Command setRightVoltage(Measure<Voltage> voltage) {
    return runOnce(
        () -> {
          // Updates input voltages.
          frontRightMotor.setVoltage(voltage.in(Volts));
          rearRightMotor.setVoltage(voltage.in(Volts));
        })
        .withName("setRightVoltage(" + voltage.in(Volts) + ")");
  }

  @Override
  public Measure<Distance> getRightDisplacement() {
    // Averages displacements of both motors.
    return Meters.of((frontRightEncoder.getPosition() + rearRightEncoder.getPosition()) / 2);
  }

  @Override
  public Measure<Velocity<Distance>> getRightVelocity() {
    // Averages velocities of both motors.
    return MetersPerSecond.of(
        (frontRightEncoder.getVelocity() + rearRightEncoder.getVelocity()) / 2);
  }

  @Override
  public void resetRightEncoder() {
    // Resets displacement measurements.
    frontRightEncoder.setPosition(0);
    rearRightEncoder.setPosition(0);
  }

  @Override
  public void resetEncoders() {
    // Resets displacement measurements.
    resetLeftEncoder();
    resetRightEncoder();
  }

  @Override
  public Measure<Velocity<Angle>> getAngularVelocity() {
    return DegreesPerSecond.of(distanceToAngle(Meters.of(getRightVelocity().in(MetersPerSecond))).in(Degrees));
  }

  @Override
  public void close() throws Exception {
    // Closes all of the motors.
    frontLeftMotor.close();
    rearLeftMotor.close();
    frontRightMotor.close();
    rearRightMotor.close();
  }

  /**
   * Creates a new instance of this DifferentialDriveIO class.
   *
   * @param motorIDs : [Front Left, Rear Left, Front Right, Rear Right]
   */
  public static SparkDiffDrive create(int[] motorIDs) {
    return new SparkDiffDrive(motorIDs);
  }

  /**
   * Creates a new instance of this DifferentialDriveIO class.
   *
   * @param motorIDs : [Front Left, Rear Left, Front Right, Rear Right]
   */
  private SparkDiffDrive(int[] motorIDs) {
    // Instantiates motors.
    this.frontLeftMotor = new CANSparkMax(motorIDs[0], MotorType.kBrushless);
    this.rearLeftMotor = new CANSparkMax(motorIDs[1], MotorType.kBrushless);
    this.frontRightMotor = new CANSparkMax(motorIDs[2], MotorType.kBrushless);
    this.rearRightMotor = new CANSparkMax(motorIDs[3], MotorType.kBrushless);

    // Resets configuration.
    this.frontLeftMotor.restoreFactoryDefaults();
    this.rearLeftMotor.restoreFactoryDefaults();
    this.frontRightMotor.restoreFactoryDefaults();
    this.rearRightMotor.restoreFactoryDefaults();

    // Sets the Idle mode to brake.
    this.frontLeftMotor.setIdleMode(IdleMode.kBrake);
    this.rearLeftMotor.setIdleMode(IdleMode.kBrake);
    this.frontRightMotor.setIdleMode(IdleMode.kBrake);
    this.rearRightMotor.setIdleMode(IdleMode.kBrake);

    // Burns configuration to flash.
    this.frontLeftMotor.burnFlash();
    this.rearLeftMotor.burnFlash();
    this.frontRightMotor.burnFlash();
    this.rearRightMotor.burnFlash();

    // Instantiates encoders.
    this.frontLeftEncoder = frontLeftMotor.getEncoder();
    this.rearLeftEncoder = rearLeftMotor.getEncoder();
    this.frontRightEncoder = frontRightMotor.getEncoder();
    this.rearRightEncoder = rearRightMotor.getEncoder();

    // Instantiates Pose estimation.
    pose = STARTING_POSE;
    prevDisplacements = new double[] { 0, 0 };
  }

  @Override
  public Pose2d getPose() {
    return pose;
  }

  @Override
  public void updatePose(Measure<Time> deltaTime) {
    // Calculates displacement of each side compared to previous timestamp.
    double[] deltaDisplacements = new double[] {
        getLeftDisplacement().in(Meters) - prevDisplacements[0],
        getRightDisplacement().in(Meters) - prevDisplacements[1]
    };

    // Amount rotated since last estimated pose update.
    Rotation2d deltaRotation = new Rotation2d(
        distanceToAngle(Meters.of(deltaDisplacements[1] - deltaDisplacements[0])));

    // New robot heading.
    Rotation2d heading = pose.getRotation().plus(deltaRotation);

    // Updates angular displacement.
    deltaDisplacements[2] = heading.getDegrees();

    // Amount moved since last estimated pose update.
    Measure<Distance> deltaXTranslation = Meters.of(Math.cos((prevRotation + heading.getDegrees()) / 2));
    Measure<Distance> deltaYTranslation = Meters.of(Math.sin((prevRotation + heading.getDegrees()) / 2));

    // Updates estimated pose.
    pose.plus(new Transform2d(deltaXTranslation, deltaYTranslation, deltaRotation));
  }
}
