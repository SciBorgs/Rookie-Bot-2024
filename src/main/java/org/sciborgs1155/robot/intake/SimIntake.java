package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.intake.IntakeConstants.LENGTH;
import static org.sciborgs1155.robot.intake.IntakeConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.intake.IntakeConstants.MIN_ANGLE;
import static org.sciborgs1155.robot.intake.IntakeConstants.MOI;
import static org.sciborgs1155.robot.intake.IntakeConstants.MOTOR_GEARING;
import static org.sciborgs1155.robot.intake.IntakeConstants.STARTING_ANGLE;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimIntake implements IntakeIO {
  private final SingleJointedArmSim wristSim =
      new SingleJointedArmSim(
          LinearSystemId.createSingleJointedArmSystem(
              DCMotor.getNEO(4),
              MOI.in((Meters).mult(Meters).mult(Kilograms)),
              1.0 / MOTOR_GEARING),
          DCMotor.getNEO(4),
          1.0 / MOTOR_GEARING,
          -LENGTH.in(Meters), // the Units library in action
          MIN_ANGLE.in(Radians), // initialized as degrees, converted to radians
          MAX_ANGLE.in(Radians), // as required by the constructor
          true,
          STARTING_ANGLE.in(Radians));

  private final DCMotorSim rollerSim = new DCMotorSim(DCMotor.getNeoVortex(1), 5, 31);

  /**
   * Gets the position of the intake that is simulated
   *
   * @return The position of the simulated wrist in radians
   */
  @Override
  public double getPosition() {
    return wristSim.getAngleRads();
  }

  /**
   * Gets the velocity of the simulated wrist.
   *
   * @return The velocity of the simulated wrist in radians per second.
   */
  @Override
  public double getVelocity() {
    return wristSim.getVelocityRadPerSec();
  }

  /**
   * Set the voltage of the simulate wrist
   *
   * @param volts The voltage applied to the simulated wrist.
   */
  @Override
  public void setWristVoltage(double volts) {
    wristSim.setInputVoltage(volts);
    wristSim.update(PERIOD.in(Seconds));
  }
}
