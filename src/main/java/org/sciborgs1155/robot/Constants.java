package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;

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

  /** Magnitude of joystick input to be considered negligable. */
  public static final double DEADBAND = 0.1;

  /**
   * Speed multiplier of the Slow-Speed mode which makes it easier to make more precise movements.
   */
  public static final double SLOW_SPEED = 0.33;

  /** Speed of the Full-Speed mode which makes it easier to travel large distances. */
  public static final double FULL_SPEED = 1.0;
}
