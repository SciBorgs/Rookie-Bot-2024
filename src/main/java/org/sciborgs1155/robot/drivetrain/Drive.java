package org.sciborgs1155.robot.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.Ports.Drive.FRONT_LEFT_DRIVE;
import static org.sciborgs1155.robot.Ports.Drive.FRONT_RIGHT_DRIVE;
import static org.sciborgs1155.robot.Ports.Drive.REAR_LEFT_DRIVE;
import static org.sciborgs1155.robot.Ports.Drive.REAR_RIGHT_DRIVE;
import static org.sciborgs1155.robot.drivetrain.DriveConstants.DEADBAND;
import static org.sciborgs1155.robot.drivetrain.DriveConstants.FULL_SPEED;
import static org.sciborgs1155.robot.drivetrain.DriveConstants.MAX_VOLTAGE;
import static org.sciborgs1155.robot.drivetrain.DriveConstants.SLOW_SPEED;
import static org.sciborgs1155.robot.drivetrain.DriveConstants.STARTING_POSE;
import static org.sciborgs1155.robot.drivetrain.DriveConstants.TRACK_WIDTH;
import static org.sciborgs1155.robot.drivetrain.DriveConstants.driveConstants;
import static org.sciborgs1155.robot.drivetrain.DriveConstants.rotateConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.MotorClosedLoopController;
import org.sciborgs1155.robot.Robot;

/** Differential drivetrain subsystem */
public class Drive extends SubsystemBase implements Logged {
  /** Hardware interface */
  @Log.NT private final DriveIO hardware;

  /** Scales voltage output */
  @Log.NT private double speedMultiplier;

  /** Voltage setpoint of the left side(Volts) */
  @Log.NT private double leftVoltage;

  /** Voltage setpoint of the right side(Volts) */
  @Log.NT private double rightVoltage;

  /** Closed loop controller for 'driveDistance' command */
  @Log.NT private final MotorClosedLoopController driveDistanceController;

  /** Closed loop controller for 'rotateAngle' command */
  @Log.NT private final MotorClosedLoopController rotateAngleController;

  /** Logs a visual repreentation of the target position for autonomous commands */
  @Log.NT private Pose2d autosGoalPose;

  /** We don't have a gyro, so this has to be used with an estimated rotation */
  private final DifferentialDriveOdometry odometry;

  /** Displacements of wheels at last odometry update */
  private final DifferentialDriveWheelPositions previousWheelDisplacements;

  @Log.NT
  /** Angular velocity of the drivetrain(DegreesPerSecond) */
  private double angularVelocity = 0;

  /**
   * Updates votages in the 'leftVoltage' and 'rightVoltage' fields (to be used with joysticks ,
   * inputs range from 0 to 1)
   */
  private final DifferentialDrive inputHandler;

  /**
   * Updates the power of the motors based on joystick input(Tank Drive)
   *
   * @param leftInput : Speed, from [-1.0,1.0]
   * @param rightInput : Speed, from [-1.0,1.0]
   */
  public Command inputTank(DoubleSupplier leftInput, DoubleSupplier rightInput) {
    return run(() -> inputHandler.tankDrive(leftInput.getAsDouble(), rightInput.getAsDouble()))
        .withName("inputTank(" + leftInput.getAsDouble() + "," + rightInput.getAsDouble() + ")");
  }

  /**
   * Updates the power of the motors based on joystick input(Arcade Drive)
   *
   * @param drive : Speed, from [-1.0,1.0]
   * @param rotation : Speed, from [-1.0,1.0]
   */
  public Command inputArcade(DoubleSupplier drive, DoubleSupplier rotation) {
    return run(() -> inputHandler.arcadeDrive(drive.getAsDouble(), rotation.getAsDouble()))
        .withName("inputArcade(" + drive.getAsDouble() + "," + rotation.getAsDouble() + ")");
  }

  /** Drives a certain distance linearly using closed-loop-feedback(Meters) */
  public Command driveDistance(double distance) {
    return runOnce(
            () -> {
              // Initialized value serves as a refrence point(measurement starts at 0)
              // If you are wondering why... I have 0 clue. Motion Profiling was doing weird stuff
              // when I just used it normally
              driveDistanceController.initialize(distance, hardware.getRightDisplacement());
              autosGoalPose = getPose().plus(new Transform2d(distance, 0, new Rotation2d()));
            })
        .andThen(
            run(() -> {
                  // Measurement is the difference between the initial value and the current value
                  double outputVoltage =
                      driveDistanceController.getOutput(
                          hardware.getRightDisplacement() - driveDistanceController.getInitial());

                  leftVoltage = outputVoltage;
                  rightVoltage = outputVoltage;
                })
                .until(driveDistanceController::isDone)
                .finallyDo(
                    () -> {
                      driveDistanceController.reset();
                    }))
        .withName("driveDistance(" + distance + "m)");
  }

  /** Rotates a certain angle using closed-loop-feedback(Degrees) */
  public Command rotateAngle(double angle) {
    return runOnce(
            () -> {
              // Initialized value serves as a refrence point(measurement starts at 0)
              // If you are wondering why... I have 0 clue. Motion Profiling was doing weird stuff
              // when I just used it normally
              rotateAngleController.initialize(angle, getPose().getRotation().getDegrees());
              autosGoalPose = getPose().plus(new Transform2d(0, 0, Rotation2d.fromDegrees(angle)));
            })
        .andThen(
            run(() -> {
                  // Measurement is the difference between the initial value and the current value
                  double outputVoltage =
                      rotateAngleController.getOutput(
                          getPose().getRotation().getDegrees()
                              - rotateAngleController.getInitial());

                  leftVoltage = outputVoltage;
                  rightVoltage = -outputVoltage;
                })
                .until(rotateAngleController::isDone)
                .finallyDo(
                    () -> {
                      rotateAngleController.reset();
                    }))
        .withName("rotateAngle(" + angle + "°)");
  }

  /** Changes the voltage scale factor to either 'FULL_SPEED' or 'SLOW_SPEED' */
  public Command switchSpeedMultiplier() {
    return runOnce(
            () -> {
              if (speedMultiplier == FULL_SPEED) {
                speedMultiplier = SLOW_SPEED;
              }
              if (speedMultiplier == SLOW_SPEED) {
                speedMultiplier = FULL_SPEED;
              }
            })
        .withName("Switch Speed Multiplier : " + speedMultiplier);
  }

  /**
   * Instantiate a new {@link Drive} subsystem
   *
   * @param isSimulated : Whether to use simulated motors or SparkMaxes
   */
  public static Drive create(boolean isSimulated) {
    if (isSimulated) {
      return new Drive(new SimDrive());
    } else {
      return new Drive(
          new SparkDrive(
              new int[] {FRONT_LEFT_DRIVE, REAR_LEFT_DRIVE, FRONT_RIGHT_DRIVE, REAR_RIGHT_DRIVE}));
    }
  }

  /**
   * Instantiate a new {@link Drive} subsystem with {@link NoDrive} hardware interface(does
   * absolutely nothing)
   */
  public static Drive createDisabledDrivetrain() {
    return new Drive(new NoDrive());
  }

  private Drive(DriveIO hardwareInterface) {
    hardware = hardwareInterface;
    driveDistanceController = new MotorClosedLoopController(driveConstants);
    rotateAngleController = new MotorClosedLoopController(rotateConstants);

    inputHandler =
        new DifferentialDrive(
            (leftVoltageInput) -> leftVoltage = leftVoltageInput,
            (rightVoltageInput) -> rightVoltage = rightVoltageInput);

    inputHandler.setMaxOutput(MAX_VOLTAGE.in(Volts));
    inputHandler.setDeadband(DEADBAND);

    odometry = new DifferentialDriveOdometry(STARTING_POSE.getRotation(), 0, 0, STARTING_POSE);
    previousWheelDisplacements = new DifferentialDriveWheelPositions(0, 0);

    leftVoltage = 0;
    rightVoltage = 0;

    speedMultiplier = FULL_SPEED;

    resetDefaultCommand();

    // Disables safety warnings on simulated drivetrain.
    if (!Robot.isReal()) {
      inputHandler.setSafetyEnabled(false);
    }
  }

  /** Sets output voltage to 0 when no commands are running */
  public void resetDefaultCommand() {
    setDefaultCommand(
        runOnce(
                () -> {
                  inputHandler.tankDrive(0, 0);
                })
            .andThen(Commands.idle(this)));
  }

  /** Returns the current position of the drivetrain(translation in meters) */
  @Log.NT
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Used for testing the 'updateOdometry' method found in {@link SparkDrive} */
  public void updateOdometry(double deltaTimeSeconds) {
    // The displacement since last odometry update(as opposed to in total)
    double[] deltaDisplacementsMeters =
        new double[] {
          hardware.getLeftDisplacement() - previousWheelDisplacements.leftMeters,
          hardware.getRightDisplacement() - previousWheelDisplacements.rightMeters
        };

    // difference in displacement can be used to find a difference in orientation
    double deltaRotationDegrees =
        (deltaDisplacementsMeters[1] - deltaDisplacementsMeters[0])
            / TRACK_WIDTH.times(Math.PI).divide(360).in(Meters);

    // old rotation + delta rotation = new rotation
    double newRotation = getPose().getRotation().getDegrees() + deltaRotationDegrees;

    // Rotation has to be calculated in order to use 'DifferentialDriveOdometry'
    odometry.update(
        Rotation2d.fromDegrees(newRotation),
        hardware.getLeftDisplacement(),
        hardware.getRightDisplacement());

    // angular velocity = delta rotation / delta time
    angularVelocity = deltaRotationDegrees / deltaTimeSeconds;

    // Sets up displacements for next update
    previousWheelDisplacements.leftMeters = hardware.getLeftDisplacement();
    previousWheelDisplacements.rightMeters = hardware.getRightDisplacement();
  }

  /** Updates motor voltages and odometry(to be called periodically) */
  public void updateVoltages() {
    hardware.setLeftVoltage(rightVoltage * speedMultiplier);
    hardware.setRightVoltage(leftVoltage * speedMultiplier);

    updateOdometry(PERIOD.in(Seconds));
    hardware.update();
  }
}
