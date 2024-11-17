package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;

/**
 * Constants is a globally accessible class for storing immutable values. Every value should be
 * <code>public static final</code>.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 * @see Units
 */
public class Constants {
  /** Roborio Tick Rate */
  public static final Measure<Time> PERIOD = Seconds.of(0.02);

  /** Returns whether the robot is real or not */
  public static final boolean robotIsReal = Robot.isReal();
}
