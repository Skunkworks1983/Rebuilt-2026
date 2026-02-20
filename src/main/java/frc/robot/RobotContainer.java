// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.commands.CollectorIntake;
import frc.robot.commands.DriveAimedGoal;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IndexerIntake;
import frc.robot.commands.IndexerReverse;
import frc.robot.commands.NoDriveShootHub;
import frc.robot.commands.SimpleShoot;
import frc.robot.commands.TestPathCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.kitCollector.Collector;
import frc.robot.subsystems.kitCollector.CollectorIO;
import frc.robot.subsystems.kitCollector.CollectorTalonIO;
import frc.robot.subsystems.kitIndexer.Indexer;
import frc.robot.subsystems.kitIndexer.IndexerIO;
import frc.robot.subsystems.kitIndexer.IndexerTalonIO;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.simpleShooter.FlywheelIO;
import frc.robot.subsystems.simpleShooter.FlywheelIOSim;
import frc.robot.subsystems.simpleShooter.FlywheelIOTalonFX;
import frc.robot.subsystems.simpleShooter.SimpleShooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.sim.SimProjectile;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final SimpleShooter shooter;
  private final Indexer indexer;
  private final Collector collector;

  // Controller
  private final Joystick leftJoystick;
  private final Joystick rightJoystick;
  private final Joystick buttonJoystick;

  /**
   * Xbox Controller for simulation.
   *
   * <p>Only used when {@link Constants#controlScheme} is set to {@link
   * Constants.ControlScheme#XBOX}
   */
  private final XboxController xboxController;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final LoggedNetworkNumber pivotSetpoint =
      new LoggedNetworkNumber("Pivot Setpoint Deg", 58.);

  // Sim
  private SwerveDriveSimulation simulation = null;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.controlScheme == Constants.ControlScheme.OI) {
      leftJoystick = new Joystick(0);
      rightJoystick = new Joystick(1);
      buttonJoystick = new Joystick(2);
      xboxController = null;
    } else {
      xboxController = new XboxController(0);
      leftJoystick = null;
      rightJoystick = null;
      buttonJoystick = null;
    }
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(Constants.tunerConstants.getFrontLeft()),
                new ModuleIOTalonFX(Constants.tunerConstants.getFrontRight()),
                new ModuleIOTalonFX(Constants.tunerConstants.getBackLeft()),
                new ModuleIOTalonFX(Constants.tunerConstants.getBackRight()),
                (pose) -> {},
                (pose) -> {});

        vision =
            new Vision(
                drive, new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));

        shooter =
            new SimpleShooter(
                new FlywheelIOTalonFX(ShooterConstants.upperFlywheelId),
                new FlywheelIOTalonFX(ShooterConstants.lowerFlywheelId));

        // This is a solution to the vision-drive cross dependency issue that hurts my
        // eys, but it
        // does work
        drive.setResetVisionIMUCallback((pose) -> vision.resetCameraIMU(pose));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));

        indexer = new Indexer(new IndexerTalonIO());
        collector = new Collector(new CollectorTalonIO());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        simulation =
            new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));

        SimulatedArena.getInstance().addDriveTrainSimulation(simulation);
        drive =
            new Drive(
                new GyroIOSim(simulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(
                    Constants.tunerConstants.getFrontLeft(), simulation.getModules()[0]),
                new ModuleIOTalonFXSim(
                    Constants.tunerConstants.getFrontRight(), simulation.getModules()[1]),
                new ModuleIOTalonFXSim(
                    Constants.tunerConstants.getBackLeft(), simulation.getModules()[2]),
                new ModuleIOTalonFXSim(
                    Constants.tunerConstants.getBackRight(), simulation.getModules()[3]),
                simulation::setSimulationWorldPose,
                (pose) -> {});

        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name,
                    VisionConstants.robotToCamera0,
                    simulation::getSimulatedDriveTrainPose));
        shooter =
            new SimpleShooter(
                new FlywheelIOSim(ShooterConstants.upperFlywheelId),
                new FlywheelIOSim(ShooterConstants.lowerFlywheelId));

        indexer = new Indexer(new IndexerIO() {});
        collector = new Collector(new CollectorIO() {});

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {},
                (pose) -> {});

        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});

        shooter = new SimpleShooter(new FlywheelIO() {}, new FlywheelIO() {});

        collector = null;
        indexer = null;

        break;
    }

    // registers commands for pathplanner
    NamedCommands.registerCommand("Collector Intake", new CollectorIntake(collector));
    NamedCommands.registerCommand("Indexer Intake", new IndexerIntake(indexer));
    NamedCommands.registerCommand("No Drive Shoot Hub", new NoDriveShootHub(shooter, drive));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Sim Physics Test", new PathPlannerAuto("SimPhysTest"));

    autoChooser.addOption(
        "Shooter SysId (Quasistatic Forward)",
        shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Shooter SysId (Quasistatic Reverse)",
        shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Shooter SysId (Dynamic Reverse)", shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Shooter SysId (Dynamic Forward)", shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    JoystickButton collectorIntake;
    JoystickButton reverseIndexer;

    if (Constants.controlScheme == Constants.ControlScheme.OI) {
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -leftJoystick.getY(), // -X (used to be)
              () -> -leftJoystick.getX(), // -Y (used to be)
              () -> -rightJoystick.getX())); // Confirm if negitive or not

      collectorIntake = new JoystickButton(buttonJoystick, Constants.collectorIntakeButtonID);
      reverseIndexer = new JoystickButton(buttonJoystick, Constants.IndexerIntakeButtonID);

      Trigger aimGoal = new JoystickButton(rightJoystick, 1);
      Trigger shoot = new JoystickButton(leftJoystick, 1);
      Trigger collect = new JoystickButton(rightJoystick, 2);
      Trigger index = new JoystickButton(leftJoystick, 2);
      reverseIndexer.whileTrue(new IndexerReverse(indexer));

      // alignShot.whileTrue(
      // DriveCommands.joystickDriveAimedHub(
      // drive, shooter, () -> -leftJoystick.getY(), () -> -leftJoystick.getX()));

      collect.whileTrue(new CollectorIntake(collector));
      if (Constants.currentMode == Mode.REAL) {
        shoot.whileTrue(SimpleShoot.simpleShoot(shooter));
      } else {
        shoot.whileTrue(new InstantCommand(this::shootSimProjectile));
      }
      index.whileTrue(new IndexerIntake(indexer));
      aimGoal.whileTrue(
          new DriveAimedGoal(
              drive, () -> -leftJoystick.getY(), () -> -leftJoystick.getX(), shooter));

    } else {

      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -xboxController.getLeftY(),
              () -> -xboxController.getLeftX(),
              () -> -xboxController.getRightX()));

      collectorIntake = new JoystickButton(xboxController, 1);
      reverseIndexer = new JoystickButton(xboxController, 2);
    }

    // collectorIntake.whileTrue(new CollectorIntake(collector));
    // indexerIntake.whileTrue(new IndexerIntake(indexer));

    Logger.recordOutput("Control Scheme", Constants.controlScheme);

    TestPathCommands.publishTestPaths(drive);

    // Lock to 0° when A button is held
    // controller
    // .a()
    // .whileTrue(
    // DriveCommands.joystickDriveAtAngle(
    // drive,
    // () -> controller.getLeftY(), //- (used to be)
    // () -> -controller.getLeftX(),
    // () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    // controller
    // .b()
    // .onTrue(
    // Commands.runOnce(
    // () ->
    // drive.setPose(
    // new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
    // drive)
    // .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * This is called at robot initialization during sim
   *
   * <p>It temporarily positions the robot (See {@link #resetSimRobotPose} for actual initial robot
   * positioning) and resets the {@link SimulatedArena} to standard field state
   */
  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    simulation.setSimulationWorldPose(new Pose2d(12, 1, new Rotation2d()));

    SimulatedArena.getInstance().resetFieldForAuto();
  }

  /**
   * This method is called during the periodic cycle during sim. The simulation state is updated and
   * game pieces positions are logged for AdvantageScope visualization
   *
   * @see <a href= "https://shenzhen-robotics-alliance.github.io/maple-sim/reefscape/">MapleSim
   *     docs</a> for information on how to display game pieces in AdvantageScope
   */
  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/RobotPosition", simulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }

  /**
   * This method is called periodically during disabled so that if the alliance is changed it is
   * registered
   *
   * <p>This method sets the {@link #simulation} world pose to mirrored positions based on alliance
   * color
   */
  public void resetSimRobotPose() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      simulation.setSimulationWorldPose(new Pose2d(7.6, 6.6, new Rotation2d(Math.PI)));
    } else {
      simulation.setSimulationWorldPose(new Pose2d(10, 1.5, new Rotation2d()));
    }
  }

  public void shootSimProjectile() {
    SimProjectile.getProjectile(
        simulation.getSimulatedDriveTrainPose(),
        simulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        shooter.getForwardFlywheelLinearVelocity(),
        ShooterConstants.pivotInitialAngle);
  }

  public void setRobotPose() {
    // if (DriverStation.getAlliance().isPresent()
    // && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
    // drive.setPose(new Pose2d(7.6, 6.6, new Rotation2d()));
    // } else {
    // drive.setPose(new Pose2d(10, 1.5, new Rotation2d(Math.PI)));
    // }

  }
}
