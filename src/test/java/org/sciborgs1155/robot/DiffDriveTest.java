package org.sciborgs1155.robot;

import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.drivetrain.Drive;

public class DiffDriveTest {
  @Test
  public void initialize() {
    Drive.create(true);
  }

  @Test
  public void teleOp() {
    Drive drivetrain = Drive.create(false);
    drivetrain.inputTank(() -> 1, () -> 1).schedule();
  }

  @Test
  void dist() {}
}
