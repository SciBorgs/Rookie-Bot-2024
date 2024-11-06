package org.sciborgs1155.robot.tankdrive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.STARTING_POSE;

import org.sciborgs1155.robot.tankdrive.DriveConstants.RotationFFD;
import org.sciborgs1155.robot.tankdrive.DriveConstants.RotationPID;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import monologue.Annotations.Log;
import monologue.Logged;

/** Stores data classes of drivetrain(for logging autonomous commands.) */
public class DiffDriveAutos {
    /** Stores translational data of autonomous commands. */
    public static class TranslationAutos implements Logged {
        /** Target Pose2d(for GUI). */
        @Log.NT
        public Pose2d goalPose = STARTING_POSE;

        /** Distance to travel(Meters). */
        @Log.NT
        public double goal = 0;

        /** Starting Pose2d(for GUI). */
        @Log.NT
        public Pose2d initialPose = STARTING_POSE;

        /** Starting position. */
        @Log.NT
        public double intitial = 0;

        /** Distance from goal(Meters). */
        @Log.NT
        public double error = 0;

        /** Distance traveled(Meters). */
        @Log.NT
        public double measure = 0;

        /** Output of PID controller(MetersPerSecond). */
        @Log.NT
        public double pidOutput = 0;

        /** Output of FFD controller(Volts). */
        @Log.NT
        public double ffdOutput = 0;

        /** Output voltage(Volts). */
        @Log.NT
        public double outputVoltage = 0;
    }

    /** Stores rotational data of autonomous commands. */
    public static class RotationAutos implements Logged {
        /** Starting Pose2d(for GUI). */
        @Log.NT
        public Pose2d initialPose = STARTING_POSE;

        /** Starting orientation(Degrees). */
        @Log.NT
        public double initial = 0;

        /** Target Pose2d(for GUI). */
        @Log.NT
        public Pose2d goalPose = STARTING_POSE;

        /** Target orientation(Degrees). */
        @Log.NT
        public double goal = 0;

        /** Angular distance from initial orientation(Degrees). */
        @Log.NT
        public double measure = 0;

        /** Angular distance from goal orientation(Degrees) */
        @Log.NT
        public double error = 0;

        /** Output of PID controller(DegreesPerSecond). */
        @Log.NT
        public double pidOutput = 0;

        /** Output of FFD controller(Volts). */
        @Log.NT
        public double ffdOutput = 0;

        /** Output voltage(right side, Volts). */
        @Log.NT
        public double outputVoltage = 0;

        /** Used for measuring acceleration(DegreesPerSecond). */
        public double lastVelocity = 0;

        /** Used for measuring the passage of time. */
        public double lastTimeStamp = 0;

        /** Velocity(DegreesPerSecond). */
        @Log.NT
        public double velocity = 0;

        /** PID controller. */
        @Log.NT
        ProfiledPIDController pidController = RotationPID.getController();

        /** Initializes goal and initial pose(based on angle to rotate). */
        public void initPID(Pose2d currentPose, Measure<Angle> angle, Measure<Velocity<Angle>> angularVelocity) {
            goalPose = new Pose2d(currentPose.getTranslation(),
                    currentPose.getRotation().plus(Rotation2d.fromDegrees(angle.in(Degrees))));
            goal = goalPose.getRotation().getDegrees();

            initialPose = currentPose;
            initial = initialPose.getRotation().getDegrees();

            update(currentPose, angularVelocity);

            pidController.reset(measure);
        }

        /** Updates error, measurement, pid/ffd outputs and output voltage. */
        public void update(Pose2d currentPose, Measure<Velocity<Angle>> angularVelocity) {
            error = goalPose.getRotation().minus(currentPose.getRotation()).getDegrees();
            measure = currentPose.getRotation().getDegrees();

            pidOutput = pidController.calculate(error);

            velocity = angularVelocity.in(DegreesPerSecond);

            ffdOutput = RotationFFD.getController().calculate(pidController.getSetpoint().velocity);

            outputVoltage = (ffdOutput + pidOutput) / 2;
            lastTimeStamp = Timer.getFPGATimestamp();

        }

        /** When this is true, stop the command. */
        public boolean isAtGoal() {
            return pidController.atGoal();
        }
    }
}
