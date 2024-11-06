package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.DEADBAND;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.MOTOR_IDS;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.STARTING_POSE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.tankdrive.DiffDriveAutos.RotationAutos;

/** Differential drivetrain. */
public class DiffDrive extends SubsystemBase implements AutoCloseable, Logged {
  /** Drivetrain hardware interface. */
  private DiffDriveIO motors;

  /** Scales voltage output to the motors. */
  private double speedMultiplier = 1;

  /** Stores rotational data of autonomous commands. */
  @Log.NT
  private final RotationAutos rotationAutos = new RotationAutos();

  /** Stores motor data(for logging). */
  public static class MotorData implements Logged {
    /** Current Pose of the robot. */
    @Log.NT
    public Pose2d pose = STARTING_POSE;

    /** Left Voltage setpoint(For logging, and streamlining). */
    @Log.NT
    public double targetLeftVoltage = 0;

    /** Right Voltage setpoint(For Logging, and streamlining). */
    @Log.NT
    public double targetRightVoltage = 0;

    /** Left velocity(MetersPerSecond). */
    @Log.NT
    public double leftVelocity = 0;

    /** Right velocity(MetersPerSecond). */
    @Log.NT
    public double rightVelocity = 0;

    /** Angular velocity(DegreesPerSecond). */
    @Log.NT
    public double angularVelocity = 0;

    /** GUI displaying robot pose. */
    @Log.NT
    public Field2d field = new Field2d();
  }

  /** Stores motor data(for logging). */
  @Log.NT
  private final MotorData motorData = new MotorData();

  /** Handles Joystick input. */
  private final DifferentialDrive inputHandler = new DifferentialDrive(
      (lVoltage) -> motorData.targetLeftVoltage = lVoltage * speedMultiplier,
      (rVoltage) -> motorData.targetRightVoltage = rVoltage * speedMultiplier);

  /**
   * Updates the power of the motors based on an arbitrary power value(Tank
   * Drive).
   *
   * @param leftInput  : Power, from [-1.0,1.0].
   * @param rightInput : Power, from [-1.0,1.0].
   */
  public Command inputTank(DoubleSupplier leftInput, DoubleSupplier rightInput) {
    return runOnce(() -> inputHandler.tankDrive(leftInput.getAsDouble(), rightInput.getAsDouble()))
        .withName("inputTank(" + leftInput.getAsDouble() + "," + rightInput.getAsDouble() + ")");
  }

  /**
   * Updates the power of the motors based on an arbitrary power value(Arcade
   * Drive).
   *
   * @param drive    : Power, from [-1.0,1.0].
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
  // public Command drive(Measure<Distance> distance) {
  //   return run(() -> {

  //   })
  //       .until(() -> translationPID.atGoal())
  //       .beforeStarting(Commands.print("Driving " + distance.in(Meters) + " meters..."))
  //       .andThen(Commands.print("Drove " + distance.in(Meters) + " meters!"))
  //       .withName("drive(" + distance.in(Meters) + ")");
  // }

  /** Turns the robot to a certain orientation. */
  public Command rotateTo(Measure<Angle> orientation) {
    return rotate(orientation)
        .withName("rotateTo(" + orientation.in(Degrees) + ")");
  }

  /** Turns the robot a certain angular distance. */
  public Command rotate(Measure<Angle> angle) {
    return run(() -> {
      rotationAutos.update(getPose(), motors.getAngularVelocity());
      inputHandler.tankDrive(rotationAutos.outputVoltage, -rotationAutos.outputVoltage);
    })
        .until(rotationAutos::isAtGoal)
        .beforeStarting(
            () -> rotationAutos.initPID(getPose(), angle, motors.getAngularVelocity()),
            this)
        .beforeStarting(Commands.print("Rotating " + Math.round(angle.in(Degrees)) + " degrees..."))
        .andThen(Commands.print("Rotated " + Math.round(angle.in(Degrees)) + " degrees!"))
        .withName("rotate(" + Math.round(angle.in(Degrees)) + ")");
  }

  /** Sets the speed multiplier. */
  public void setSpeedMultiplier(double speedMultiplier) {
    this.speedMultiplier = speedMultiplier;
  }

  /** Creates an instance of this subsystem(depending on if it is real or not). */
  public static DiffDrive create() {
    if (Robot.isReal())
      return new DiffDrive(SparkDiffDrive.create(MOTOR_IDS));

    if (!Robot.isReal())
      return new DiffDrive(SimDiffDrive.create());

    return null;
  }

  /** Creates an instance of this subsystem. */
  public static DiffDrive create(boolean real) {
    if (real)
      return new DiffDrive(SparkDiffDrive.create(MOTOR_IDS));

    if (!real)
      return new DiffDrive(SimDiffDrive.create());

    return null;
  }

  private DiffDrive(DiffDriveIO io) {
    motors = io;
    motorData.field = new Field2d();

    inputHandler.setMaxOutput(DriveConstants.MAX_VOLTAGE.in(Volts));
    inputHandler.setDeadband(DEADBAND);

    resetDefaultCommand();

    // Disables safety warnings on simulated drivetrain.
    if (!Robot.isReal()) {
      inputHandler.setSafetyEnabled(false);
    }
  }

  /** Sets output voltage to 0 when no commands are running. */
  public void resetDefaultCommand() {
    setDefaultCommand(
        runOnce(
            () -> {
              inputHandler.tankDrive(0, 0);
            }));
  }

  /** Returns the current position of the drivetrain. */
  public Pose2d getPose() {
    return motors.getPose();
  }

  /** Updates motor voltages and odometry(to be called periodically). */
  public void updateVoltages() {
    motors.setLeftVoltage(Volts.of(motorData.targetLeftVoltage)).schedule();
    motors.setRightVoltage(Volts.of(motorData.targetRightVoltage)).schedule();

    motorData.leftVelocity = motors.getLeftVelocity().in(MetersPerSecond);
    motorData.rightVelocity = motors.getRightVelocity().in(MetersPerSecond);
    motorData.angularVelocity = motors.getAngularVelocity().in(DegreesPerSecond);

    motors.updatePose(PERIOD);
    motorData.pose = getPose();
    motorData.field.setRobotPose(motorData.pose);
  }

  /** Closes field, inputHandler, and motors. */
  @Override
  public void close() throws Exception {
    motorData.field.close();
    inputHandler.close();
    motors.close();
  }
}
