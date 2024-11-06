package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.tankdrive.DriveConstants.WHEEL_BASE;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.tankdrive.DiffDrive;

public class DiffDriveTest {
    /** Simulated differential drivetrain. */
    DiffDrive simDiffDrive;

    @BeforeEach
    public void reset() {
        simDiffDrive = DiffDrive.create(false);
    }

    @AfterEach
    public void destroy() throws Exception {
        simDiffDrive.close();
    }

    @Test
    void velocityTest() throws Exception {
        System.out.println(Degrees.of(5 / WHEEL_BASE.times(Math.PI).divide(360).in(Meters)));
    }
}
