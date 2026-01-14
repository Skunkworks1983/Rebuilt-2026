package frc.robot.commands;

import frc.robot.util.LoggedTunableNumber;

public class TestPathCommandConstants {
  public static LoggedTunableNumber maxVelocityMPS =
      new LoggedTunableNumber("TestPaths/Max Velocity m/s", 3.0);
  public static LoggedTunableNumber maxAccelerationMPSPS =
      new LoggedTunableNumber("TestPaths/Max Acceleration m/s^2", 4.0);
  public static LoggedTunableNumber maxAngularVelocityDPS =
      new LoggedTunableNumber("TestPaths/Max Angular Velocity deg/s", 540);
  public static LoggedTunableNumber maxAngularAccelerationDPSPS =
      new LoggedTunableNumber("TestPaths/Max Angular Acceleration deg/s^2", 720);
}
