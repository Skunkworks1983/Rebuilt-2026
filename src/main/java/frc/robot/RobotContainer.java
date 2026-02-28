// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TestPathCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private enum Songs {
    RickRoll,
    Megalovania,
    Charlie_Kirk
  }

  private enum Hardware{
    kitbot,
    actual
  }

  public static final Hardware CURRENT_HARDWARE = Hardware.kitbot;
  
  private final String DRIVEBASE_CANBUS;
  private static final String RIO_CANBUS = "rio";

  private final int frontLeftDriveId;
  private final int frontLeftSteerId;
  private final int frontRightDriveId;
  private final int frontRightSteerId;
  private final int backLeftDriveId;
  private final int backLeftSteerId;
  private final int backRightDriveId;
  private final int backRightSteerId;

  private final int forwardShootId;
  private final int backShootId;
  private final int indexerId;
  private final int collectorId;
  private final int pivotMotorId;

  Orchestra m_orchestra = new Orchestra();
  private final Notifier orchestraReplayNotifier = new Notifier(() -> m_orchestra.play());

  LoggedDashboardChooser<Songs> songChooser = new LoggedDashboardChooser<>("Song Chooser");

  private final TalonFX flDrive;
  private final TalonFX flSteer;
  private final TalonFX frDrive;
  private final TalonFX frSteer;
  private final TalonFX blDrive;
  private final TalonFX blSteer;
  private final TalonFX brDrive;
  private final TalonFX brSteer;

  private final TalonFX forwardShoot;
  private final TalonFX backShoot;
  private final TalonFX indexer;
  private final TalonFX collector;
  private final TalonFX pivotMotor;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (CURRENT_HARDWARE == Hardware.actual) {
      frontLeftDriveId = 8;
      frontLeftSteerId = 10;
      frontRightDriveId = 11;
      frontRightSteerId = 13;
      backLeftDriveId = 14;
      backLeftSteerId = 16;
      backRightDriveId = 19;
      backRightSteerId = 17;

      forwardShootId = 23;
      backShootId = 21;
      indexerId = 45;
      collectorId = 46;
      pivotMotorId = 47;

      DRIVEBASE_CANBUS = "drivebase_2026";
    } else {
      frontLeftDriveId = 10;
      frontLeftSteerId = 11;
      frontRightDriveId = 13;
      frontRightSteerId = 14;
      backLeftDriveId = 16;
      backLeftSteerId = 17;
      backRightDriveId = 20;
      backRightSteerId = 19;

      forwardShootId = 42;
      backShootId = 43;
      indexerId = 44;
      collectorId = 45;
      pivotMotorId=46;
      DRIVEBASE_CANBUS = "Drivebase 2025";
    }

    flDrive = new TalonFX(frontLeftDriveId, DRIVEBASE_CANBUS);
    flSteer = new TalonFX(frontLeftSteerId, DRIVEBASE_CANBUS);
    frDrive = new TalonFX(frontRightDriveId, DRIVEBASE_CANBUS);
    frSteer = new TalonFX(frontRightSteerId, DRIVEBASE_CANBUS);
    blDrive = new TalonFX(backLeftDriveId, DRIVEBASE_CANBUS);
    blSteer = new TalonFX(backLeftSteerId, DRIVEBASE_CANBUS);
    brDrive = new TalonFX(backRightDriveId, DRIVEBASE_CANBUS);
    brSteer = new TalonFX(backRightSteerId, DRIVEBASE_CANBUS);

    forwardShoot = new TalonFX(forwardShootId, RIO_CANBUS);
    backShoot = new TalonFX(backShootId, RIO_CANBUS);
    indexer = new TalonFX(indexerId, RIO_CANBUS);
    collector = new TalonFX(collectorId, RIO_CANBUS);
    pivotMotor = new TalonFX(pivotMotorId, RIO_CANBUS);

    Command crazyCommand = new InstantCommand(this::playMusic).ignoringDisable(true);
    songChooser.addOption("Rick Roll", Songs.RickRoll);
    songChooser.addOption("Megalovania", Songs.Megalovania);
    songChooser.addOption("We are Charlie Kirk", Songs.Charlie_Kirk);
    SmartDashboard.putData("Do Thing!", crazyCommand);
  }

  public void playMusic() {
    AudioConfigs config = new AudioConfigs().withAllowMusicDurDisable(true);

    flDrive.getConfigurator().apply(config);
    flSteer.getConfigurator().apply(config);
    frDrive.getConfigurator().apply(config);
    frSteer.getConfigurator().apply(config);
    blDrive.getConfigurator().apply(config);
    blSteer.getConfigurator().apply(config);
    brDrive.getConfigurator().apply(config);
    brSteer.getConfigurator().apply(config);

    forwardShoot.getConfigurator().apply(config);
    backShoot.getConfigurator().apply(config);
    indexer.getConfigurator().apply(config);
    collector.getConfigurator().apply(config);

    // Melody Track (0)
    m_orchestra.addInstrument(forwardShoot, 0);
    m_orchestra.addInstrument(backShoot, 0);
    m_orchestra.addInstrument(collector, 0);
    m_orchestra.addInstrument(indexer, 0);
    m_orchestra.addInstrument(flDrive, 0);
    m_orchestra.addInstrument(frDrive, 0);
    m_orchestra.addInstrument(blDrive, 0);
    m_orchestra.addInstrument(blSteer, 0);
    m_orchestra.addInstrument(brDrive, 0);
    m_orchestra.addInstrument(brSteer, 0);
    m_orchestra.addInstrument(pivotMotor, 0);

    // Bass track (1)
    m_orchestra.addInstrument(flSteer, 1);
    m_orchestra.addInstrument(frSteer, 1);
    m_orchestra.addInstrument(blSteer, 1);
    m_orchestra.addInstrument(brSteer, 1);

    switch (songChooser.get()) {
      case RickRoll:
        m_orchestra.loadMusic("rickroll.chrp");
        break;
      case Megalovania:
        m_orchestra.loadMusic("megalovania.chrp");
        break;
      case Charlie_Kirk:
        m_orchestra.loadMusic("kirk.chrp");
        break;
      default:
        m_orchestra.loadMusic("megalovania.chrp");
        break;
    }

    m_orchestra.play();

    orchestraReplayNotifier.stop();
    orchestraReplayNotifier.startPeriodic(120.0);
  }

}
