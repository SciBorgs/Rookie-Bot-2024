package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.DEADBAND;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.MOTOR_IDS;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.tankdrive.DriveConstants.DriveFFD;
import org.sciborgs1155.robot.tankdrive.DriveConstants.DrivePID;
import org.sciborgs1155.robot.tankdrive.DriveConstants.RotationFFD;
import org.sciborgs1155.robot.tankdrive.DriveConstants.RotationPID;

/** Differential drivetrain. */
public class TankDrive extends SubsystemBase implements AutoCloseable, Logged {
  /** Drivetrain hardware interface. */
  private DifferentialDriveIO motors;

  /** Handles Joystick input. */
  private DifferentialDrive inputHandler;

  /** Allows for manual control of the speed of the motors. */
  public double speedMultiplier;

  /** Voltage setpoints. */
  private DifferentialDriveWheelVoltages targetVoltages;

  @Log.NT
  /** Position setpoint. */
  private Pose2d goalPose;

  @Log.NT
  /** Error. */
  private double error;

  @Log.NT
  /** Output of PID controller. */
  private double pidOutput;

  @Log.NT
  /** Output of FFD controller. */
  private double ffdOutput;

  @Log.NT
  /** GUI displaying robot pose. */
  private Field2d field;

  /** PID controller for driving(linear error -> linear velocity). */
  private ProfiledPIDController drivePID;

  /** PID controller for rotating(angular error -> angular velocity). */
  private ProfiledPIDController rotationPID;

  /** FFD controller for all driving(linear velocity -> voltage). */
  private SimpleMotorFeedforward driveFFD;

  /** FFD controller for all rotating(angular velocity -> voltage). */
  private SimpleMotorFeedforward rotationFFD;

  /**
   * Updates the power of the motors based on an arbitrary power value(Tank Drive).
   *
   * @param leftInput : Power, from [-1.0,1.0].
   * @param rightInput : Power, from [-1.0,1.0].
   */
  public Command input(DoubleSupplier leftInput, DoubleSupplier rightInput) {
    return runOnce(() -> inputHandler.tankDrive(leftInput.getAsDouble(), rightInput.getAsDouble()))
        .withName("input(" + leftInput.getAsDouble() + "," + rightInput.getAsDouble() + ")");
  }

  /**
   * Updates the power of the motors based on an arbitrary power value(Arcade Drive).
   *
   * @param drive : Power, from [-1.0,1.0].
   * @param rotation : Power, from [-1.0,1.0].
   */
  public Command inputArcade(DoubleSupplier drive, DoubleSupplier rotation) {
    return runOnce(() -> inputHandler.arcadeDrive(-drive.getAsDouble(), -rotation.getAsDouble()))
        .withName("inputArcade(" + drive.getAsDouble() + "," + rotation.getAsDouble() + ")");
  }

  /**
   * Drives a certain distance.
   *
   * @param distance : Distance.
   * @return Command.
   */
  public Command drive(Measure<Distance> distance) {
    System.out.println("Driving " + distance.in(Meters) + " meters!");
    // Calculates the target pose after the command is done.
    goalPose = getPose().plus(new Transform2d(distance, Meters.of(0), Rotation2d.fromDegrees(0)));

    // Runs closed loop until within tolerance of the target position.
    return run(() -> {
          // Distance from goal pose(Meters).
          error = getPose().getTranslation().getDistance(goalPose.getTranslation());

          // PID calculations(error -> velocity(MetersPerSecond)).
          pidOutput = drivePID.calculate(error, 0);

          // FFD calculations(velocity -> voltage(Volts)).
          if (distance.gt(Meters.of(0))) {
            ffdOutput = -driveFFD.calculate(pidOutput);
          }
          if (distance.lt(Meters.of(0))) {
            ffdOutput = driveFFD.calculate(pidOutput);
          }

          // Updates voltages.
          inputHandler.tankDrive(ffdOutput, ffdOutput);
        })
        .until(() -> drivePID.atGoal())
        .withName("drive(" + distance.in(Meters) + ")");
  }

  /** Turns the robot a certain angle. */
  public Command rotate(Measure<Angle> angle) {
    return rotateTo(Radians.of(angle.in(Radians) + getPose().getRotation().getRadians()))
        .withName("rotate(" + angle.in(Radians) + ")");
  }

  /** Turns the robot to a certain orientation. */
  public Command rotateTo(Measure<Angle> angle) {
    System.out.println("Rotating to " + Math.round(angle.in(Degrees)) + " degrees!");
    // The target pose after the command is done(current pose).
    goalPose =
        getPose()
            .plus(
                new Transform2d(
                    Meters.of(0), Meters.of(0), Rotation2d.fromRadians(angle.in(Radians))));

    return run(() -> {
          // Angular distance from goal(radians)
          error = getPose().getRotation().minus(goalPose.getRotation()).getRadians();

          // PID calculations(error -> velocity).
          pidOutput = rotationPID.calculate(error, 0);

          // FFD calculations(velocity -> voltage).
          ffdOutput = rotationFFD.calculate(Units.radiansToDegrees(pidOutput));

          // Updates voltages.
          inputHandler.tankDrive(-ffdOutput, ffdOutput);
        })
        .until(() -> rotationPID.atGoal())
        .withName("rotateTo(" + angle.in(Radians) + ")");
  }

  /**
   * Sets the speed multiplier of the drivetrain.
   *
   * @param speedMultiplier : Amount to multiply the voltage by.
   */
  public void setSpeedMultiplier(double speedMultiplier) {
    this.speedMultiplier = speedMultiplier;
  }

  /**
   * Creates an instance of tankdrive depending on if the robot is real or not.
   *
   * @return Instance of Tankdrive.
   */
  public static TankDrive create() {
    if (Robot.isReal()) {
      return new TankDrive(SparkDiffDrive.create(MOTOR_IDS));
    }
    if (!Robot.isReal()) {
      return new TankDrive(SimDiffDrive.create());
    }
    return null;
  }

  private TankDrive(DifferentialDriveIO hardware) {
    // Instantiation of hardware.
    this.motors = hardware;

    // Allows InputHandler to interact with motors.
    targetVoltages = new DifferentialDriveWheelVoltages(0, 0);
    inputHandler =
        new DifferentialDrive(
            (lVoltage) -> targetVoltages.left = lVoltage * speedMultiplier,
            (rVoltage) -> targetVoltages.right = rVoltage * speedMultiplier);

    // Scales output to voltage.
    inputHandler.setMaxOutput(DriveConstants.MAX_VOLTAGE.in(Volts));

    // Point at which joystick input can be considered negligable.
    inputHandler.setDeadband(DEADBAND);

    // Instantiation of remaining fields.
    speedMultiplier = 0.2;
    goalPose = new Pose2d();
    field = new Field2d();

    // Disables safety warnings on sim drivetrain.
    if (!Robot.isReal()) {
      inputHandler.setSafetyEnabled(false);
    }

    // Instantiates PID and FFD controller.
    drivePID = DrivePID.getController();
    driveFFD = DriveFFD.getController();
    rotationPID = RotationPID.getController();
    rotationFFD = RotationFFD.getController();

    // Default command is to stop.
    resetDefaultCommand();
  }

  /** Resets the default command to stop. */
  public void resetDefaultCommand() {
    setDefaultCommand(
        runOnce(
            () -> {
              inputHandler.tankDrive(driveFFD.calculate(0), driveFFD.calculate(0));
            }));
  }

  /** Returns the current position of the drivetrain. */
  public Pose2d getPose() {
    return motors.getPose();
  }

  @Override
  public void periodic() {
    // Updates voltages.
    motors.setLeftVoltage(targetVoltages.left);
    motors.setRightVoltage(targetVoltages.right);

    // Updates Field2d position.
    motors.updatePose(PERIOD);
    field.setRobotPose(motors.getPose());
  }

  @Override
  public void close() throws Exception {
    // Closes everything.
    field.close();
    inputHandler.close();
    motors.close();
  }
}
