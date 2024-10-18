package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
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

import java.util.function.DoubleSupplier;

import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.tankdrive.DriveConstants.DriveFFD;
import org.sciborgs1155.robot.tankdrive.DriveConstants.DrivePID;
import org.sciborgs1155.robot.tankdrive.DriveConstants.RotationFFD;
import org.sciborgs1155.robot.tankdrive.DriveConstants.RotationPID;
import org.sciborgs1155.robot.tankdrive.TankModuleIO.NoModule;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;

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

  @Log.NT
  /** Odometry field GUI. */
  private Field2d odofield;

  /** PID controller for driving(linear error -> linear velocity). */
  @Log.NT
  private ProfiledPIDController drivePID = new ProfiledPIDController(DrivePID.P, DrivePID.I, DrivePID.D,
      DRIVE_CONSTRAINTS);

  /** PID controller for rotating(angular error -> angular velocity). */
  @Log.NT
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
    return runOnce(() -> inputHandler.arcadeDrive(-drive.getAsDouble(), rotation.getAsDouble()))
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
    goalPose = odometry
        .getPoseMeters()
        .plus(new Transform2d(Meters.of(2), Meters.of(0), Rotation2d.fromDegrees(0)));

    SmartDashboard.putNumber("Drivetrain Setpoint", 0);

    // Runs closed loop until within tolerance of the target position.
    return run(
        () -> {
          SmartDashboard.putNumber("Drivetrain Error",
              odometry.getPoseMeters().getTranslation().getDistance(goalPose.getTranslation()));

          // PID calculations(error -> velocity).
          Measure<Velocity<Distance>> pidOutput = MetersPerSecond
              .of(drivePID.calculate(odometry.getPoseMeters().getTranslation().getDistance(goalPose.getTranslation()),
                  0));
          SmartDashboard.putNumber("Drivetrain PID Output", pidOutput.in(MetersPerSecond));

          // FFD calculations(velocity -> voltage, negated because PID is weird).
          Measure<Voltage> ffdOutput = Volts.of(-driveFFD.calculate(pidOutput.in(MetersPerSecond)));

          // Updates voltages.
          inputHandler.tankDrive(ffdOutput.in(Volts), ffdOutput.in(Volts));
        }).until(() -> drivePID.atGoal()).withName("drive(" + distance.in(Meters) + ")");
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
    // The goal end pose of the robot after the command has been ran.
    Rotation2d goalPose = Rotation2d.fromRadians(angle.in(Radians));

    return run(
        () -> {
          // The PID measurement.
          Measure<Angle> distanceFromGoal = Radians
              .of(odometry.getPoseMeters().getRotation().minus(goalPose).getRadians());

          // PID calculations(error -> velocity).
          Measure<Velocity<Angle>> pidOutput = RadiansPerSecond
              .of(rotationPID.calculate(distanceFromGoal.in(Radians), 0));

          // FFD calculations(velocity -> voltage).
          Measure<Voltage> ffdOutput = Volts.of(rotationFFD.calculate(pidOutput.in(RadiansPerSecond)));

          // Updates voltages.
          inputHandler.tankDrive(ffdOutput.in(Volts), -ffdOutput.in(Volts));
        }).withName("rotateTo(" + angle.in(Radians) + ")");
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
   * Updates Odometry measurements.
   */
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
    lastDisplacement = new DifferentialDriveWheelPositions(left.getDisplacementDouble(), right.getDisplacementDouble());

    // Updates the robot pose using displacements and calculated heading.
    odometry.update(fakeGyro, left.getDisplacement().in(Meters), right.getDisplacement().in(Meters));

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
        (lVoltage) -> {
          targetVoltages.left = lVoltage * speedMultiplier;
          left.setVoltage(targetVoltages.left);
        },
        (rVoltage) -> {
          targetVoltages.right = rVoltage * speedMultiplier;
          right.setVoltage(targetVoltages.right);
        });

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
    odometry = new DifferentialDriveOdometry(fakeGyro, lastDisplacement.leftMeters, lastDisplacement.rightMeters);
    odofield = new Field2d();

    // Maximum speed.
    speedMultiplier = 1;

    // Disables garbage ahh warnings.
    inputHandler.setSafetyEnabled(false);
    inputHandler.setDeadband(DEADBAND);

    // Default command is to stop.
    setDefaultCommand(runOnce(() -> {
      inputHandler.tankDrive(0, 0);
    }));
  }

  /** Resets the default command to stop. */
  public void resetDefaultCommand() {
    setDefaultCommand(runOnce(() -> {
      inputHandler.tankDrive(0, 0);
    }));
  }

  @Override
  public void periodic() {
    // Updates odometry.
    updateOdometry();

    // Updates voltages on real and sim motors.
    inputHandler.tankDrive(targetVoltages.left, targetVoltages.right);
    simulation.setInputs(targetVoltages.left, targetVoltages.right);

    // Updates simulation.
    simulation.update(PERIOD.in(Seconds));
    simfield.setRobotPose(simulation.getPose());
  }

  @Override
  public void close() throws Exception {
    this.close();
  }
}
