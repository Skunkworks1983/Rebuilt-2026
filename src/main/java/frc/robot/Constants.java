// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import frc.robot.generated.TunerConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = Mode.REAL;
  public static final ControlScheme controlScheme = ControlScheme.OI;
  public static final RobotHardware robotHardware = RobotHardware.RebuiltChassisOne;
  public static final TunerConstants tunerConstants = TunerConstants.getTunerConstants();

  public static final boolean tuningMode = true;
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
    RebuiltChassisOne,
    RebuiltChassisTwo
  }

  /** OI constants */
  public final class OIConstants {
    // Joystick ID's
    public static final int LeftJoystick = 0;
    public static final int RightJoystick = 1;
    public static final int ButtonJoystick = 2;

    // Joystick Button ID's
    public static final int joystickTrigger = 1; // left
    public static final int joystickAutoAim = 1; // right

    // Collector ID's
    public static final int CollectorIntakeID = 24;
    public static final int CollectorReverseIntakeID = 22;
    public static final int CollectorPivotDown = 4;
    public static final int CollectorPivotUp = 12;

    // Shooter button joystick ID's
    public static final int ShooterShootHubID = 15;
    public static final int ShooterShootTransferID = 13;
    public static final int ShooterTriggerID = 24;
    public static final int ShooterReverseTriggerID = 17;

    // public static final int ShooterFlywheelSwitchID = 6;
    public static final int ShooterHoldToAimID = 1; // Jotstick trigger
    public static final int ShooterReverseIndexID = 18;

    public static final int dontAgitate = 3;

    public static final int gamepadCollect = 2;
    public static final int gamepadCollectorReverse = 1;
  }
}
