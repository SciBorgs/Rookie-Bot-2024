// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package org.sciborgs1155.robot.roller;

import static org.sciborgs1155.robot.Constants.isReal;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Roller extends SubsystemBase {

    private final RollerIO hardware;
    public static final double ROLLER_MAX = .4;
    public static final double ROLLER_REVERSE = -.05;
/**
 * Creates a Roller (Constructor).
 * @param hardware
 */
    private Roller(RollerIO hardware) {
        this.hardware = hardware;
    }

    /**
     * Creates a real intake or a no intake.
     * @return The constructor for a real or no intake.
     */
    public static Roller create() {
        return isReal() ? new Roller(new RealRoller()) : new Roller(new NoRoller());
    }

    /**
     * A command that sets the the roller.
     * @param voltage Voltage applied to the roller.
     * @return A command that sets the the roller.
     */
    private Command setRoller(double voltage) {
        return run(() -> hardware.setRoller(voltage));
    }

    public Command roller() {
        return setRoller(ROLLER_MAX);
    }

    public Command stopRoller() {
        return setRoller(0);
    }

    public Command reverseRoller() {
        return setRoller(ROLLER_REVERSE);
    }
}
