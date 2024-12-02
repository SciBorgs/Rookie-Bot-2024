package org.sciborgs1155.robot;

import static org.sciborgs1155.robot.drivetrain.DriveConstants.driveConstants;

import org.junit.jupiter.api.Test;
import org.sciborgs1155.lib.MotorClosedLoopController;
import org.sciborgs1155.robot.drivetrain.Drive;

public class DriveTest {
  @Test
  void initialize() {
    Drive.create(true);
  }

  @Test
  void teleOp() {
    Drive drive = Drive.create(true);
    drive.inputTank(() -> 1, () -> 1).schedule();
  }

  @Test
  void controller() {
    int tick = 0;
    final int maxticks = 1000;

    double measure = 0;
    final double starting = 0;
    final double goal = 20;

    MotorClosedLoopController controller = new MotorClosedLoopController(driveConstants);

    controller.initialize(goal, starting);
    while (tick < maxticks) {
      System.out.println(
          "Error: "
              + controller.getError()
              + " Measure: "
              + measure
              + " Output: "
              + controller.getOutput(measure));
      measure += controller.getOutput(measure);
      tick++;
    }
  }
}
