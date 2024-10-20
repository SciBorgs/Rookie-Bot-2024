package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.teleop;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.test;
import static org.sciborgs1155.robot.Constants.PERIOD;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import monologue.Annotations.Log;
import monologue.Logged;
import monologue.Monologue;
import org.littletonrobotics.urcl.URCL;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.commands.Autos;
import org.sciborgs1155.robot.tankdrive.TankDrive;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot extends CommandRobot implements Logged {

  // INPUT DEVICES
  @SuppressWarnings("unused")
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);

  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);

  // SUBSYSTEMS
  @Log.NT
  private final TankDrive drive = TankDrive.create();

  // COMMANDS
  @Log.NT
  private final Autos autos = new Autos();

  /** The robot contains subsystems, OI devices, and commands. */
  public Robot() {
    configureGameBehavior();
    configureSubsystemDefaults();
    configureBindings();
  }

  /** Configures basic behavior during different parts of the game. */
  private void configureGameBehavior() {
    // Configure logging with DataLogManager, Monologue, FailureManagement, and URCL
    DataLogManager.start();
    Monologue.setupMonologue(this, "/Robot", false, true);
    addPeriodic(Monologue::updateAll, PERIOD.in(Seconds));
    FaultLogger.setupLogging();
    addPeriodic(FaultLogger::update, PERIOD.in(Seconds));
    if (!isReal()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
    if (isReal()) {
      URCL.start();
    }
  }

  /**
   * Configures subsystem default commands. Default commands are scheduled when no
   * other command is
   * running on a subsystem.
   */
  private void configureSubsystemDefaults() {
  }

  /** Configures trigger -> command bindings */
  private void configureBindings() {

    teleop()
        .onTrue(
            Commands.runOnce(
                () -> {
                  CommandScheduler.getInstance().cancelAll();
                  drive.setDefaultCommand(
                      drive.inputArcade(() -> driver.getRawAxis(1), () -> driver.getRawAxis(0)));
                  System.out.println("Enabled Teleop!");
                }));

    teleop()
        .onFalse(
            Commands.runOnce(
                () -> {
                  drive.resetDefaultCommand();
                  System.out.println("Disabled Teleop!");
                }));

    test()
        .onTrue(
            Commands.runOnce(
                () -> {
                  drive.resetDefaultCommand();

                  System.out.println("Enabled Test Mode!");
                  drive.drive(Meters.of(10)).finallyDo(() -> {
                    drive.rotate(Degrees.of(135)).schedule();
                  }).schedule();
                }));

    FaultLogger.onFailing(fault -> Commands.print(fault.toString()));

    driver
        .leftBumper()
        .or(driver.rightBumper())
        .onTrue(Commands.runOnce(() -> drive.speedMultiplier = Constants.FULL_SPEED))
        .onFalse(Commands.run(() -> drive.speedMultiplier = Constants.SLOW_SPEED));
  }
}
