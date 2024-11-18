package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.Constants.robotIsReal;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.MAX_VOLTAGE;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.MOTOR_IDS;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.SLOW_SPEED;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.tankdrive.DriveConstants.DiffDrivetrainLogger;

/** Differential drivetrain. */
public class DiffDrive extends SubsystemBase implements Logged {
  /** Hardware interface */
  private DiffDriveIO motors;

  /** Scales voltage output */
  private double speedMultiplier = SLOW_SPEED;

  /** Stores motor data(for logging and streamlining voltage setting) */
  @Log.NT private final DiffDrivetrainLogger motorData = new DiffDrivetrainLogger();

  /** Updates votages in 'motorData' class(with speedMultiplier) */
  private final DifferentialDrive inputHandler =
      new DifferentialDrive(
          (leftVoltage) -> motorData.targetLeftVoltage = leftVoltage * speedMultiplier,
          (rightVoltage) -> motorData.targetRightVoltage = rightVoltage * speedMultiplier);

  /**
   * Updates the power of the motors based on joystick input (Tank Drive)
   *
   * @param leftInput : Power, from [-1.0,1.0]
   * @param rightInput : Power, from [-1.0,1.0]
   */
  public Command inputTank(DoubleSupplier leftInput, DoubleSupplier rightInput) {
    return run(() -> inputHandler.tankDrive(leftInput.getAsDouble(), rightInput.getAsDouble()))
        .withName("inputTank(" + leftInput.getAsDouble() + "," + rightInput.getAsDouble() + ")");
  }

  /**
   * Updates the power of the motors based on joystick input (Arcade Drive)
   *
   * @param drive : Power, from [-1.0,1.0]
   * @param rotation : Power, from [-1.0,1.0]
   */
  public Command inputArcade(DoubleSupplier drive, DoubleSupplier rotation) {
    return run(() -> inputHandler.arcadeDrive(drive.getAsDouble(), rotation.getAsDouble()))
        .withName("inputArcade(" + drive.getAsDouble() + "," + rotation.getAsDouble() + ")");
  }

  /** Updates the voltage scale factor */
  public void setSpeedMultiplier(double speedMultiplier) {
    this.speedMultiplier = speedMultiplier;
  }

  /** Return a new {@link DiffDrive} class(depending on if it is a sim or not) */
  public static DiffDrive create(boolean isSimulated) {
    if (!isSimulated) return new DiffDrive(new SparkDiffDrive(MOTOR_IDS));

    if (isSimulated) return new DiffDrive(new SimDiffDrive());

    return null;
  }

  private DiffDrive(DiffDriveIO hardwareInterface) {
    motors = hardwareInterface;
    inputHandler.setMaxOutput(MAX_VOLTAGE.in(Volts));
    resetDefaultCommand();

    // Disables safety warnings on simulated drivetrain.
    if (!robotIsReal) {
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
  public Pose2d getPose() {
    return motors.getPose();
  }

  /** Updates motor voltages and odometry(to be called periodically) */
  public void updateVoltages() {
    motors.setLeftVoltage(Volts.of(motorData.targetLeftVoltage));
    motors.setRightVoltage(Volts.of(motorData.targetRightVoltage));

    motorData.leftVelocity = motors.getLeftVelocity().in(MetersPerSecond);
    motorData.rightVelocity = motors.getRightVelocity().in(MetersPerSecond);
    motorData.angularVelocity = motors.getAngularVelocity().in(DegreesPerSecond);

    motors.updatePose(PERIOD);

    motorData.pose = getPose();
    motorData.field.setRobotPose(motorData.pose);
  }
}
