package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MatchHelpers {
  private static Alliance alliance;
  private static Alliance firstInactive;

  /**
   * Uses FMS data to get if the hub is currently active for scoring. This is based on the current
   * match time
   *
   * <p>Warning: The FMS does not send an official match time. When connected to the real field,
   * this number only changes in full integer second increments. This differs from when in practice,
   * which is a floating point number. This means that this function is not testable.
   *
   * @return is the hub active for the robot's current alliance?
   */
  public static boolean getIsHubActive() {
    double matchTime = DriverStation.getMatchTime();

    // Transition Shift
    if (matchTime >= 130) {
      return true;
    }

    // Shift 1
    if (130 >= matchTime && matchTime >= 105 && alliance != firstInactive) {
      return true;
    }

    // Shift 2
    if (105 >= matchTime && matchTime >= 80 && alliance == firstInactive) {
      return true;
    }

    // Shift 3
    if (80 >= matchTime && matchTime >= 55 && alliance != firstInactive) {
      return true;
    }

    // Shift 4
    if (55 >= matchTime && matchTime >= 30 && alliance == firstInactive) {
      return true;
    }

    // End Game
    return true;
  }

  public static void teleopInit() {
    String gameData;
    gameData = DriverStation.getGameSpecificMessage();

    alliance =
        DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Blue
            ? Alliance.Blue
            : Alliance.Red;

    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          firstInactive = Alliance.Blue;
          break;
        case 'R':
          firstInactive = Alliance.Red;
          break;
      }
    }
  }
}
