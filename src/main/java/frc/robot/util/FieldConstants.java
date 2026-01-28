package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

public class FieldConstants {
  public static final Translation3d blueGoal =
      new Translation3d(Inches.of(182.11), Inches.of(158.845), Inches.of(72));
  public static final Translation3d redGoal =
      new Translation3d(Inches.of(469.09), Inches.of(158.845), Inches.of(72));

  public static final Translation2d redFuelTransferTarget =
      new Translation2d(Inches.of(162.11), Inches.of(158.845));
  public static final Translation2d blueFuelTransferTarget = new Translation2d();

  public static final Translation2d blueHubCenter = new Translation2d();
  public static final Translation2d redHubCenter =
      new Translation2d(Inches.of(479.09), Inches.of(158.845));
  public static final Distance halfHubWidth = Inches.of(23.5);
}
