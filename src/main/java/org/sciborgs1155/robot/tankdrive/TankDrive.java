package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
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
import static org.sciborgs1155.robot.Constants.PERIOD;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Annotations.Log;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.tankdrive.DriveConstants.DriveFFD;
import org.sciborgs1155.robot.tankdrive.DriveConstants.DrivePID;
import org.sciborgs1155.robot.tankdrive.DriveConstants.RotationFFD;
import org.sciborgs1155.robot.tankdrive.DriveConstants.RotationPID;

public class TankDrive extends SubsystemBase implements AutoCloseable, Logged {
  /** Left side of the drivetrain. */
  private TankModuleIO left;

  /** Right side of the drivetrain. */
  private TankModuleIO right;

  /** Handles Joystick input. */
  private DifferentialDrive inputHandler;

  /** Simulated drivetrain. */
  private DifferentialDrivetrainSim simulation;

  /** "Gyroscope" used to track rotation for odometry measurements. */
  private Rotation2d fakeGyro;

  /** Used for odometry. */
  private DifferentialDriveWheelPositions lastPositions;

  /** Used for tracking robot position. */
  private DifferentialDriveOdometry odometry;

  /** Simulated field GUI. */
  @Log.NT
  private Field2d simfield;

  /** Odometry field GUI. */
  @Log.NT
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
   * Creates an instance of tankdrive depending on if the robot is real or not.
   *
   * @return T A N K.
   */
  public static TankDrive create() {
    if (Robot.isReal()) {
      return new TankDrive(
          SparkModule.create(FRONT_LEFT_DRIVE, REAR_LEFT_DRIVE, "Left Module"),
          SparkModule.create(FRONT_RIGHT_DRIVE, REAR_RIGHT_DRIVE, "Right Module"));
    }
    if (!Robot.isReal()) {
      return new TankDrive(SimModule.create("Left Module"), SimModule.create("Right Module"));
    }

    return null;
  }

  private TankDrive(TankModuleIO left, TankModuleIO right) {
    // Instantiation.
    this.left = left;
    this.right = right;
    inputHandler = new DifferentialDrive(left::setVoltage, right::setVoltage);

    // Scales output to voltage.
    inputHandler.setMaxOutput(DriveConstants.MAX_VOLTAGE.in(Volts));

    // Instantiation of the sim.
    simulation = new DifferentialDrivetrainSim(
        DCMotor.getNEO(2), // 2 NEO motors on each side of the drivetrain.
        REDUCTION,
        MOI_MASS.in(Kilograms),
        ROBOT_MASS.in(Kilograms),
        WHEEL_RADIUS.in(Meters),
        TRACK_WIDTH.in(Meters),
        STD_DEVS);
    simfield = new Field2d();

    // Instantiates odometry.
    lastPositions = new DifferentialDriveWheelPositions(0, 0);
    fakeGyro = new Rotation2d(Degrees.of(0));
    odometry = new DifferentialDriveOdometry(fakeGyro, lastPositions.leftMeters, lastPositions.rightMeters);
    odofield = new Field2d();

    // Default command.
    setDefaultCommand(Commands.idle(this));
    SmartDashboard.putData(this);
  }

  /**
   * Updates the power of the motors based on an arbitrary power value(tank).
   *
   * @param leftInput  : Power, from [-1.0,1.0].
   * @param rightInput : Power, from [-1.0,1.0].
   * @return Command.
   */
  public Command input(DoubleSupplier leftInput, DoubleSupplier rightInput) {
    return runOnce(() -> {
      inputHandler.tankDrive(leftInput.getAsDouble(), rightInput.getAsDouble());
    }).withTimeout(PERIOD.in(Seconds))
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
    return runOnce(() -> {
      inputHandler.arcadeDrive(drive.getAsDouble(), rotation.getAsDouble());
    }).withTimeout(PERIOD.in(Seconds))
        .withName("inputArcade(" + drive.getAsDouble() + "," + rotation.getAsDouble() + ")");
  }

  /**
   * Drives a certain distance.
   *
   * @param distance : Distance.
   * @return Command.
   */
  public Command drive(Measure<Distance> distance) {
    // The goal end pose of the robot after the command has been ran.
    Translation2d goalPose = odometry
        .getPoseMeters()
        .transformBy(
            new Transform2d(Meters.of(0), distance, odometry.getPoseMeters().getRotation()))
        .getTranslation();

    // The PID measurement.
    Measure<Distance> distanceFromGoal = Meters.of(odometry.getPoseMeters().getTranslation().getDistance(goalPose));

    return run(
        () -> {
          Measure<Velocity<Distance>> pidOutput = MetersPerSecond
              .of(drivePID.calculate(distanceFromGoal.in(Meters), 0));
          Measure<Voltage> ffdOuput = Volts.of(driveFFD.calculate(pidOutput.in(MetersPerSecond)));

          CommandScheduler.getInstance()
              .schedule(left.setVoltage(ffdOuput).alongWith(right.setVoltage(ffdOuput)));
        }).withName("drive(" + distance.in(Meters) + ")");
  }

  /**
   * Turns a certain angle.
   *
   * @param angle : Angle
   * @return Command.
   */
  public Command rotate(Measure<Angle> angle) {
    return rotateTo(angle.plus(Radians.of(odometry.getPoseMeters().getRotation().getRadians())))
        .withName("rotate(" + angle.in(Radians) + ")");
  }

  /**
   * Turns to a certain orientation.
   *
   * @param angle : Angle
   * @return Command.
   */
  public Command rotateTo(Measure<Angle> angle) {
    // The goal end pose of the robot after the command has been ran.
    Rotation2d goalPose = Rotation2d.fromRadians(angle.in(Radians));

    // The PID measurement.
    Measure<Angle> distanceFromGoal = Radians.of(odometry.getPoseMeters().getRotation().minus(goalPose).getRadians());

    return run(
        () -> {
          Measure<Velocity<Angle>> pidOutput = RadiansPerSecond
              .of(rotationPID.calculate(distanceFromGoal.in(Radians), 0));
          Measure<Voltage> ffdOuput = Volts.of(rotationFFD.calculate(pidOutput.in(RadiansPerSecond)));

          CommandScheduler.getInstance()
              .schedule(left.setVoltage(ffdOuput).alongWith(right.setVoltage(ffdOuput)));
        }).withName("rotateTo(" + angle.in(Radians) + ")");
  }

  @Override
  public void periodic() {
    // Differences in positions
    DifferentialDriveWheelPositions deltaPositions = new DifferentialDriveWheelPositions(
        left.getPosition().in(Meters) - lastPositions.leftMeters,
        right.getPosition().in(Meters) - lastPositions.rightMeters);

    // Amount of distance traveled non-linear-ly
    Measure<Distance> distanceRotated = Meters.of(deltaPositions.leftMeters - deltaPositions.rightMeters);

    // Converts distance to angle.
    Measure<Angle> angleRotated = Radians.of(distanceRotated.in(Meters) * TRACK_WIDTH.in(Meters) * 2 / Math.PI);

    // Updates the fake gyro based on angle rotated.
    fakeGyro = fakeGyro.rotateBy(Rotation2d.fromRadians(angleRotated.in(Radians)));
    lastPositions = new DifferentialDriveWheelPositions(left.getPositionDouble(), right.getPositionDouble());

    // Updates the odometry.
    odometry.update(fakeGyro, left.getPosition().in(Meters), right.getPosition().in(Meters));

    // Updates sim GUI's.
    simulation.setInputs(left.getVoltageDouble(), right.getVoltageDouble());
    simfield.setRobotPose(simulation.getPose());
    odofield.setRobotPose(odometry.getPoseMeters());
  }

  @Override
  public void close() throws Exception {
    this.close();
  }
}
