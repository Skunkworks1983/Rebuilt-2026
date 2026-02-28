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

  private static final String DRIVEBASE_CANBUS = "drivebase_2026";
  private static final String RIO_CANBUS = "rio";

  

  private static final int FRONT_LEFT_DRIVE_ID = 8;
  private static final int FRONT_LEFT_STEER_ID = 10;
  private static final int FRONT_RIGHT_DRIVE_ID = 11;
  private static final int FRONT_RIGHT_STEER_ID = 13;
  private static final int BACK_LEFT_DRIVE_ID = 14;
  private static final int BACK_LEFT_STEER_ID = 16;
  private static final int BACK_RIGHT_DRIVE_ID = 19;
  private static final int BACK_RIGHT_STEER_ID = 17;

  private static final int FORWARD_SHOOT_ID = 23;
  private static final int BACK_SHOOT_ID = 21;
  private static final int INDEXER_ID = 45;
  private static final int COLLECTOR_ID = 46;

  Orchestra m_orchestra = new Orchestra();
  private final Notifier orchestraReplayNotifier = new Notifier(() -> m_orchestra.play());

  LoggedDashboardChooser<Songs> songChooser = new LoggedDashboardChooser<>("Song Chooser");

  private final TalonFX flDrive = new TalonFX(FRONT_LEFT_DRIVE_ID, DRIVEBASE_CANBUS);
  private final TalonFX flSteer = new TalonFX(FRONT_LEFT_STEER_ID, DRIVEBASE_CANBUS);
  private final TalonFX frDrive = new TalonFX(FRONT_RIGHT_DRIVE_ID, DRIVEBASE_CANBUS);
  private final TalonFX frSteer = new TalonFX(FRONT_RIGHT_STEER_ID, DRIVEBASE_CANBUS);
  private final TalonFX blDrive = new TalonFX(BACK_LEFT_DRIVE_ID, DRIVEBASE_CANBUS);
  private final TalonFX blSteer = new TalonFX(BACK_LEFT_STEER_ID, DRIVEBASE_CANBUS);
  private final TalonFX brDrive = new TalonFX(BACK_RIGHT_DRIVE_ID, DRIVEBASE_CANBUS);
  private final TalonFX brSteer = new TalonFX(BACK_RIGHT_STEER_ID, DRIVEBASE_CANBUS);

  private final TalonFX forwardShoot = new TalonFX(FORWARD_SHOOT_ID, RIO_CANBUS);
  private final TalonFX backShoot = new TalonFX(BACK_SHOOT_ID, RIO_CANBUS);
  private final TalonFX indexer = new TalonFX(INDEXER_ID, RIO_CANBUS);
  private final TalonFX collector = new TalonFX(COLLECTOR_ID, RIO_CANBUS);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
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
