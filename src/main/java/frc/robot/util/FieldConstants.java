package frc.robot.util;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class FieldConstants {
  public static final Translation3d blueGoal = new Translation3d(Meters.of(0), Meter.of(0), Meters.of(1));
  public static final Translation3d redGoal = new Translation3d(Meters.of(5), Meters.of(5), Meters.of(1));
  public static final Translation2d redFuelTransferTarget = new Translation2d();
  public static final Translation2d blueFuelTransferTarget = new Translation2d();
}
