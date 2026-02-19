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

  public static interface HubCorners {
    Translation2d getAllianceSideLeft();

    Translation2d getAllianceSideRight();

    Translation2d getNeutralSideLeft();

    Translation2d getNeutralSideRight();
  }

  public static class BlueHub implements HubCorners {

    public static final Translation2d ALLIANCE_LEFT =
        new Translation2d(Inches.of(156.61), Inches.of(182.2325));
    public static final Translation2d ALLIANCE_RIGHT =
        new Translation2d(Inches.of(156.61), Inches.of(135.4475));
    public static final Translation2d NEUTRAL_LEFT =
        new Translation2d(Inches.of(215.34616), Inches.of(187.86));
    public static final Translation2d NEUTRAL_RIGHT =
        new Translation2d(Inches.of(215.34616), Inches.of(129.82));

    @Override
    public Translation2d getAllianceSideLeft() {
      return ALLIANCE_LEFT;
    }

    @Override
    public Translation2d getAllianceSideRight() {
      return ALLIANCE_RIGHT;
    }

    @Override
    public Translation2d getNeutralSideLeft() {
      return NEUTRAL_LEFT;
    }

    @Override
    public Translation2d getNeutralSideRight() {
      return NEUTRAL_RIGHT;
    }
  }

  public static class RedHub implements HubCorners {

    public static final Translation2d ALLIANCE_LEFT =
        new Translation2d(Inches.of(493.51), Inches.of(182.2325));
    public static final Translation2d ALLIANCE_RIGHT =
        new Translation2d(Inches.of(493.51), Inches.of(135.4475));
    public static final Translation2d NEUTRAL_LEFT =
        new Translation2d(Inches.of(434.77), Inches.of(187.86));
    public static final Translation2d NEUTRAL_RIGHT =
        new Translation2d(Inches.of(434.77), Inches.of(129.82));

    @Override
    public Translation2d getAllianceSideLeft() {
      return ALLIANCE_LEFT;
    }

    @Override
    public Translation2d getAllianceSideRight() {
      return ALLIANCE_RIGHT;
    }

    @Override
    public Translation2d getNeutralSideLeft() {
      return NEUTRAL_LEFT;
    }

    @Override
    public Translation2d getNeutralSideRight() {
      return NEUTRAL_RIGHT;
    }
  }

  public static final HubCorners blueHubCorners = new BlueHub();
  public static final HubCorners redHubCorners = new RedHub();
}
