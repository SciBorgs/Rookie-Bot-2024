package org.sciborgs1155.robot.drivetrain;

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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

/** Differential drivetrain subsystem */
public class Drive extends SubsystemBase implements Logged {
  /** Hardware interface */
  @Log.NT private DriveIO hardware;

  /** Scales voltage output */
  @Log.NT private double speedMultiplier = FULL_SPEED;

  /** Voltage setpoint of the left side(Volts) */
  @Log.NT private double leftVoltage = 0;

  /** Voltage setpoint of the right side(Volts) */
  @Log.NT private double rightVoltage = 0;

  /**
   * Updates votages in the 'leftVoltage' and 'rightVoltage' fields (to be used with joysticks ,
   * inputs range from 0 to 1)
   */
  private final DifferentialDrive inputHandler =
      new DifferentialDrive(
          (leftVoltageInput) -> leftVoltage = leftVoltageInput,
          (rightVoltageInput) -> rightVoltage = rightVoltageInput);

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
    if (!isSimulated)
      return new Drive(
          new SparkDrive(
              new int[] {FRONT_LEFT_DRIVE, REAR_LEFT_DRIVE, FRONT_RIGHT_DRIVE, REAR_RIGHT_DRIVE}));

    if (isSimulated) return new Drive(new SimDrive());

    return null;
  }

  /**
   * Instantiate a new {@link Drive} subsystem with {@link NoDrive} hardware interface(to be used as
   * a placeholder)
   */
  public static Drive createPlaceholder() {
    return new Drive(new NoDrive());
  }

  private Drive(DriveIO hardwareInterface) {
    hardware = hardwareInterface;

    inputHandler.setMaxOutput(MAX_VOLTAGE.in(Volts));
    inputHandler.setDeadband(DEADBAND);

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

  /** Returns the current position of the drivetrain */
  @Log.NT
  public Pose2d getPose() {
    return hardware.getPose();
  }

  /** Updates motor voltages and odometry(to be called periodically) */
  public void updateVoltages() {
    hardware.setLeftVoltage(rightVoltage * speedMultiplier);
    hardware.setRightVoltage(leftVoltage * speedMultiplier);

    hardware.updatePose(PERIOD.in(Seconds));
  }
}
