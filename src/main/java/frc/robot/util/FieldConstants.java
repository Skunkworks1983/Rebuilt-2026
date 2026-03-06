package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

public class FieldConstants {
  private static final Distance FIELD_LENGTH_INCHES = Inches.of(651.25);
  private static final Distance FIELD_WIDTH_INCHES = Inches.of(317.69);
  private static final Distance ALLIANCE_ZONE_LENGTH_INCHES = Inches.of(182.11);

  public static final Translation3d blueGoal =
      new Translation3d(Inches.of(182.11), Inches.of(158.845), Inches.of(72));
  public static final Translation3d redGoal =
      new Translation3d(Inches.of(469.09), Inches.of(158.845), Inches.of(72));

  public static final Translation2d redFuelTransferTarget =
      new Translation2d(Inches.of(162.11), Inches.of(158.845)); // TODO :SET ACTUAL

  public static final Translation2d blueFuelTransferTarget =
      new Translation2d(Inches.of(142.11), Inches.of(158.845));

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

  /** Alliance zone rectangle centered on each alliance wall. */
  public record AllianceZone(Translation2d minCorner, Translation2d maxCorner) {
    public boolean contains(Translation2d point) {
      return point.getX() >= minCorner.getX()
          && point.getX() <= maxCorner.getX()
          && point.getY() >= minCorner.getY()
          && point.getY() <= maxCorner.getY();
    }
  }

  public static final AllianceZone blueAllianceZone =
      new AllianceZone(
          new Translation2d(Inches.of(0.0), Inches.of(0.0)),
          new Translation2d(ALLIANCE_ZONE_LENGTH_INCHES, FIELD_WIDTH_INCHES));

  public static final AllianceZone redAllianceZone =
      new AllianceZone(
          new Translation2d(
              Inches.of(FIELD_LENGTH_INCHES.in(Inches) - ALLIANCE_ZONE_LENGTH_INCHES.in(Inches)),
              Inches.of(0.0)),
          new Translation2d(FIELD_LENGTH_INCHES, FIELD_WIDTH_INCHES));
}
