package org.sciborgs1155.robot;

import org.junit.jupiter.api.Test;
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
}
