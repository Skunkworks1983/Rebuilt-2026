// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final ControlScheme controlScheme = ControlScheme.OI;
  public static final RobotHardware robotHardware = RobotHardware.Rebuilt;
  public static final TunerConstants tunerConstants = TunerConstants.getTunerConstants();

  public static final boolean tuningMode = false;
  public static final boolean disableHAL = false;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum ControlScheme {
    OI,
    XBOX
  }

  public static enum RobotHardware {
    Swerveles,
    Reefscape,
    Rebuilt
  }

  /** OI constants */
  /** OI constants */
  public final class OIConstants {
    // Joystick ID's
    public static final int LeftJoystick = 0;
    public static final int RightJoystick = 1;
    public static final int ButtonJoystick = 2;

    // Collector button joystick ID's
    public static final int CollectorManualModeID = 4;
    public static final int CollectorAutoModeID = 2;

    public static final int CollectorIntakeID = 23;
    public static final int CollectorRightDownID = 11;
    public static final int CollectorLeftDownID = 12;
    public static final int CollectorReverseIntakeID = 24;

    // Shooter button joystick ID's
    public static final int ShooterShootHubID = 1;
    public static final int ShooterShootTransferID = 6;
    
    public static final int ShooterIndexID = 18;
    public static final int ShooterFlywheelSwitchID = 3;
    public static final int ShooterHoldToAimID = 1; // Jotstick trigger
    public static final int ShooterReverseIndexID = 17;
  }
}
