package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public final class MatchHelpers {
  private static Optional<Alliance> alliance = Optional.empty();
  private static Optional<Alliance> firstInactive = Optional.empty();

  private static final double TRANSITION_SHIFT_END = 130.0;
  private static final double ENDGAME_START = 30.0;
  private static final double SHIFT_LENGTH = 25.0;

  public static boolean getIsHubActive() {
    double t = DriverStation.getMatchTime();

    // Transition shift
    if (t >= TRANSITION_SHIFT_END) {
      return true;
    }

    // Endgame / last 30 seconds
    if (t <= ENDGAME_START) {
      return true;
    }

    if (alliance.isEmpty() || firstInactive.isEmpty()) {
      return false;
    }

    // [105,130) shiftIndex 0
    // [80,105)  shiftIndex 1
    // [55,80)   shiftIndex 2
    // [30,55)   shiftIndex 3
    int shiftIndex = (int) ((TRANSITION_SHIFT_END - t) / SHIFT_LENGTH);

    boolean isShiftOdd = (shiftIndex % 2 == 1);
    return isShiftOdd
        ? alliance.get() == firstInactive.get()
        : alliance.get() != firstInactive.get();
  }

  public static void teleopInit() {
    alliance = DriverStation.getAlliance();
    String gameData = DriverStation.getGameSpecificMessage();

    if (gameData != null && !gameData.isEmpty()) {
      char c = gameData.charAt(0);
      if (c == 'B') firstInactive = Optional.of(Alliance.Blue);
      else if (c == 'R') firstInactive = Optional.of(Alliance.Red);
      else firstInactive = Optional.empty();
    } else {
      firstInactive = Optional.empty();
    }
  }
}
