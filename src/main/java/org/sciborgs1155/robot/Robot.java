package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.disabled;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.teleop;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.test;
import static org.sciborgs1155.robot.Constants.PERIOD;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import monologue.Logged;
import monologue.Monologue;
import org.littletonrobotics.urcl.URCL;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.drivetrain.DiffDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot extends CommandRobot implements Logged {

  /** Controls intake, shooter, and climber */
  @SuppressWarnings("unused")
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);

  /** Controls drivetrain */
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);

  private final DiffDrive drivetrain = DiffDrive.create(isSimulation());

  /** The robot contains subsystems, OI devices, and commands. */
  public Robot() {
    configureGameBehavior();
    configureBindings();
  }

  /** Configures basic behavior during different parts of the game. */
  private void configureGameBehavior() {
    DataLogManager.start();

    Monologue.setupMonologue(this, "/Robot", false, true);
    addPeriodic(Monologue::updateAll, PERIOD.in(Seconds));

    FaultLogger.setupLogging();
    FaultLogger.onFailing(fault -> Commands.print(fault.toString()));
    addPeriodic(FaultLogger::update, PERIOD.in(Seconds));

    addPeriodic(drivetrain::updateVoltages, PERIOD.in(Seconds));

    if (!isReal()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
    if (isReal()) {
      URCL.start();
    }
  }

  /** Configures command bindings */
  private void configureBindings() {
    teleop().onTrue(teleopCommand());
    test().onTrue(testCommand());
    autonomous().onTrue(autonomousCommand());
    disabled().onTrue(disabledCommand());

    System.out.println("Configured Command Bindings!");
  }

  /** Runs once when teleop mode is enabled. Binded to 'teleop' trigger. */
  private Command teleopCommand() {
    CommandScheduler.getInstance().cancelAll();

    return Commands.sequence(
            Commands.print("Enabled Teleop Mode!"),
            drivetrain.inputArcade(() -> driver.getLeftY(), () -> driver.getRightX()))
        .withName("Teleop Command")
        .finallyDo(() -> System.out.println("Disabled Teleop Mode!"));
  }

  /** Runs once when test mode is enabled. Binded to 'test' trigger. */
  private Command testCommand() {
    CommandScheduler.getInstance().cancelAll();

    return Commands.sequence(
            Commands.print("Enabled Test Mode!"),
            drivetrain.inputTank(() -> driver.getLeftY(), () -> driver.getRightY()))
        .withName("Test Command")
        .finallyDo(() -> System.out.println("Disabled Test Mode!"));
  }

  /** Runs once when autonomous mode is enabled. Binded to 'autonomous' trigger. */
  private Command autonomousCommand() {
    CommandScheduler.getInstance().cancelAll();

    return Commands.sequence(Commands.print("Enabled Autonomous Mode!"))
        .withName("Autonomous Command")
        .finallyDo(() -> System.out.println("Disabled Autonomous Mode!"));
  }

  /** Runs once when robot is disabled. Binded to 'disabled' trigger. */
  private Command disabledCommand() {
    CommandScheduler.getInstance().cancelAll();

    return Commands.sequence(Commands.print("Disabled Robot!")).withName("Disabled Command");
  }
}
