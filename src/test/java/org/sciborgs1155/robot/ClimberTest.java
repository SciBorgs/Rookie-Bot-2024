package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.climber.Climber;

public class ClimberTest {

  Climber climber;

  @BeforeEach
  public void setup() {
    setupHAL();
    climber = Climber.create();
  }

  @Test
  public void moveToGoal() {
    Measure<Distance> goal = Meters.of(Units.inchesToMeters(4));
    run(climber.moveToGoal(goal));
    fastForward();
    assertEquals(
        goal.in(Meters), climber.height().in(Meters), 5E-3); // (BEFORE) 1E-5-tolerance precision! :D 
  }
}
