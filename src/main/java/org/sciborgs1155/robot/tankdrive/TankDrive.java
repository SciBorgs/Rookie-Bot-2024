package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.DEADBAND;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.Ports.Drive.FRONT_LEFT_DRIVE;
import static org.sciborgs1155.robot.Ports.Drive.FRONT_RIGHT_DRIVE;
import static org.sciborgs1155.robot.Ports.Drive.REAR_LEFT_DRIVE;
import static org.sciborgs1155.robot.Ports.Drive.REAR_RIGHT_DRIVE;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.DRIVE_CONSTRAINTS;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.MOI_MASS;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.REDUCTION;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.ROBOT_MASS;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.ROTATION_CONSTRAINTS;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.STD_DEVS;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.TRACK_WIDTH;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.WHEEL_RADIUS;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import org.sciborgs1155.robot.tankdrive.TankModuleIO.NoModule;

/** Differential Drive class with odometry and simulation. */
public class TankDrive extends SubsystemBase implements AutoCloseable, Logged {
  /** Left side of the drivetrain. */
  private TankModuleIO left;

  /** Right side of the drivetrain. */
  private TankModuleIO right;

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

  /** Stores robot heading. */
  private Rotation2d fakeGyro;

  /** Displacement of motors since last odometry update. */
  private DifferentialDriveWheelPositions lastDisplacement;

  /** Used for tracking robot position. */
  private DifferentialDriveOdometry odometry;

  /** Simulated drivetrain. */
  private DifferentialDrivetrainSim simulation;

  @Log.NT
  /** Simulated field GUI. */
  private Field2d simfield;

  /** Odometry field GUI. */
  private Field2d odofield;

  /** PID controller for driving(linear error -> linear velocity). */
  private ProfiledPIDController drivePID = new ProfiledPIDController(DrivePID.P, DrivePID.I, DrivePID.D,
      DRIVE_CONSTRAINTS);

  /** PID controller for rotating(angular error -> angular velocity). */
  private ProfiledPIDController rotationPID = new ProfiledPIDController(RotationPID.P, RotationPID.I, RotationPID.D,
      ROTATION_CONSTRAINTS);

  /** FFD controller for all driving(linear velocity -> voltage). */
  private SimpleMotorFeedforward driveFFD = new SimpleMotorFeedforward(DriveFFD.S, DriveFFD.V, DriveFFD.A);

  /** FFD controller for all rotating(angular velocity -> voltage). */
  private SimpleMotorFeedforward rotationFFD = new SimpleMotorFeedforward(RotationFFD.S, RotationFFD.V, RotationFFD.A);

  /**
   * Updates the power of the motors based on an arbitrary power value(tank).
   *
   * @param leftInput  : Power, from [-1.0,1.0].
   * @param rightInput : Power, from [-1.0,1.0].
   * @return Command.
   */
  public Command input(DoubleSupplier leftInput, DoubleSupplier rightInput) {
    return runOnce(() -> inputHandler.tankDrive(leftInput.getAsDouble(), rightInput.getAsDouble()))
        .withTimeout(PERIOD.in(Seconds))
        .withName("input(" + leftInput.getAsDouble() + "," + rightInput.getAsDouble() + ")");
  }

  /**
   * Updates the power of the motors based on an arbitrary power value(arcade).
   *
   * @param drive    : Power, from [-1.0,1.0].
   * @param rotation : Power, from [-1.0,1.0].
   * @return Command.
   */
  public Command inputArcade(DoubleSupplier drive, DoubleSupplier rotation) {
    return runOnce(() -> inputHandler.arcadeDrive(-drive.getAsDouble(), -rotation.getAsDouble()))
        .withTimeout(PERIOD.in(Seconds))
        .withName("inputArcade(" + drive.getAsDouble() + "," + rotation.getAsDouble() + ")");
  }

  /**
   * Drives a certain distance.
   *
   * @param distance : Distance.
   * @return Command.
   */
  public Command drive(Measure<Distance> distance) {
    // The target pose after the command is done(current pose).
    Transform2d transform = new Transform2d(distance, Meters.of(0), Rotation2d.fromDegrees(0));

    if (Robot.isReal()) {
      goalPose = odometry.getPoseMeters().plus(transform);
    }

    if (!Robot.isReal()) {
      goalPose = simulation.getPose().plus(transform);
    }

    // Runs closed loop until within tolerance of the target position.
    return run(() -> {
      if (Robot.isReal()) {
        // Meters.
        error = (odometry.getPoseMeters().getTranslation().getDistance(goalPose.getTranslation()));
      }
      if (!Robot.isReal()) {
        // Meters.
        error = (simulation.getPose().getTranslation().getDistance(goalPose.getTranslation()));
      }

      // PID calculations(error -> velocity(MetersPerSecond)).
      pidOutput = drivePID.calculate(error, 0);

      // FFD calculations(velocity -> voltage(Volts)).
      if (distance.gt(Meters.of(0))) {
        ffdOutput = -driveFFD.calculate(pidOutput);
        ;
      }
      if (distance.lt(Meters.of(0))) {
        ffdOutput = driveFFD.calculate(pidOutput);
        ;
      }

      // Updates voltages.
      inputHandler.tankDrive(ffdOutput, ffdOutput);
    })
        .until(() -> drivePID.atGoal())
        .withName("drive(" + distance.in(Meters) + ")");
  }

  /**
   * Turns a certain angle.
   *
   * @param angle : Angle.
   * @return Command.
   */
  public Command rotate(Measure<Angle> angle) {
    return rotateTo(angle.plus(Radians.of(odometry.getPoseMeters().getRotation().getRadians())))
        .withName("rotate(" + angle.in(Radians) + ")");
  }

  /**
   * Turns to a certain orientation.
   *
   * @param angle : Angle.
   * @return Command.
   */
  public Command rotateTo(Measure<Angle> angle) {
    // The target pose after the command is done(current pose).
    Transform2d transform = new Transform2d(Meters.of(0), Meters.of(0), Rotation2d.fromRadians(angle.in(Radians)));

    if (Robot.isReal()) {
      goalPose = odometry.getPoseMeters().plus(transform);
    }

    if (!Robot.isReal()) {
      goalPose = simulation.getPose().plus(transform);
    }

    return run(() -> {
      if (Robot.isReal()) {
        // Radians.
        error = odometry.getPoseMeters().getRotation().minus(goalPose.getRotation()).getRadians();
      }
      if (!Robot.isReal()) {
        // Radians.
        error = simulation.getPose().getRotation().minus(goalPose.getRotation()).getRadians();
      }

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

  /** Updates Odometry measurements. */
  public void updateOdometry() {
    // Differences in wheel displacement since last odometry update.
    DifferentialDriveWheelPositions deltaPositions = new DifferentialDriveWheelPositions(
        left.getDisplacement().in(Meters) - lastDisplacement.leftMeters,
        right.getDisplacement().in(Meters) - lastDisplacement.rightMeters);

    // Amount of distance traveled non-linear-ly.
    Measure<Distance> distanceRotated = Meters.of(deltaPositions.leftMeters - deltaPositions.rightMeters);

    // Converts distance to angle.
    Measure<Angle> angleRotated = Radians.of(distanceRotated.in(Meters) * TRACK_WIDTH.in(Meters) * 2 / Math.PI);

    // Adds angular displacement to previous heading.
    fakeGyro = fakeGyro.rotateBy(Rotation2d.fromRadians(angleRotated.negate().in(Radians)));

    // Updates the displacement.
    lastDisplacement = new DifferentialDriveWheelPositions(
        left.getDisplacementDouble(), right.getDisplacementDouble());

    // Updates the robot pose using displacements and calculated heading.
    odometry.update(
        fakeGyro, left.getDisplacement().in(Meters), right.getDisplacement().in(Meters));

    // Displays updated robot pose.
    odofield.setRobotPose(odometry.getPoseMeters());
  }

  /**
   * Creates an instance of tankdrive depending on if the robot is real or not.
   *
   * @return Instance of Tankdrive.
   */
  public static TankDrive create() {
    if (Robot.isReal()) {
      return new TankDrive(
          SparkModule.create(FRONT_LEFT_DRIVE, REAR_LEFT_DRIVE),
          SparkModule.create(FRONT_RIGHT_DRIVE, REAR_RIGHT_DRIVE));
    }
    if (!Robot.isReal()) {
      return new TankDrive(NoModule.create(), NoModule.create());
    }

    return null;
  }

  private TankDrive(TankModuleIO left, TankModuleIO right) {
    // Instantiation of hardware and interfaces.
    this.left = left;
    this.right = right;
    targetVoltages = new DifferentialDriveWheelVoltages(0, 0);
    goalPose = new Pose2d();
    inputHandler = new DifferentialDrive(
        (lVoltage) -> targetVoltages.left = lVoltage * speedMultiplier,
        (rVoltage) -> targetVoltages.right = rVoltage * speedMultiplier);

    // Scales output to voltage.
    inputHandler.setMaxOutput(DriveConstants.MAX_VOLTAGE.in(Volts));

    // Instantiation of the sim.
    simulation = new DifferentialDrivetrainSim(
        DCMotor.getNEO(2),
        REDUCTION,
        MOI_MASS.in(Kilograms),
        ROBOT_MASS.in(Kilograms),
        WHEEL_RADIUS.in(Meters),
        TRACK_WIDTH.in(Meters),
        STD_DEVS);
    simfield = new Field2d();

    // Instantiates odometry.
    lastDisplacement = new DifferentialDriveWheelPositions(0, 0);
    fakeGyro = new Rotation2d(Degrees.of(0));
    odometry = new DifferentialDriveOdometry(
        fakeGyro, lastDisplacement.leftMeters, lastDisplacement.rightMeters);
    odofield = new Field2d();

    // Maximum speed.
    speedMultiplier = 0.2;

    // Point at which joystick input can be considered negligable.
    inputHandler.setDeadband(DEADBAND);

    // Disables safety warnings on sim drivetrain.
    if (!Robot.isReal()) {
      inputHandler.setSafetyEnabled(false);
    }

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

  @Override
  public void periodic() {
    // Logs stuff
    SmartDashboard.putNumber("Drivetrain Error", error);
    SmartDashboard.putNumber("Drivetrain PID Output", pidOutput);
    SmartDashboard.putNumber("Drivetrain FFD Output", ffdOutput);

    // Updates voltages on real and sim motors.
    inputHandler.tankDrive(targetVoltages.left, targetVoltages.right);
    simulation.setInputs(targetVoltages.left, targetVoltages.right);
    left.setVoltage(targetVoltages.left);
    right.setVoltage(targetVoltages.right);

    // Updates simulation.
    simulation.update(PERIOD.in(Seconds));
    simfield.setRobotPose(simulation.getPose());
  }

  @Override
  public void close() throws Exception {
    this.close();
  }
}
