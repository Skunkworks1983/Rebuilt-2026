// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveAimedAutoShoot;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Shooter.AutonomousAimedShoot;
import frc.robot.commands.Shooter.NoYawShootHub;
import frc.robot.commands.Shooter.ShootPresetSetpoints;
import frc.robot.commands.Shooter.ShootPresetSetpoints.TargetType;
import frc.robot.commands.TestPathCommands;
import frc.robot.commands.collectorCommands.doubleCollectorCommands.BothCollectorsUpState;
import frc.robot.commands.collectorCommands.doubleCollectorCommands.CollectorExpel;
import frc.robot.commands.collectorCommands.doubleCollectorCommands.CollectorIntake;
import frc.robot.commands.collectorCommands.doubleCollectorCommands.HoldPosition;
import frc.robot.commands.collectorCommands.doubleCollectorCommands.LeftCollectorDownState;
import frc.robot.commands.collectorCommands.doubleCollectorCommands.ManualCollectorDown;
import frc.robot.commands.collectorCommands.doubleCollectorCommands.ManualCollectorUp;
import frc.robot.commands.collectorCommands.doubleCollectorCommands.RightCollectorDownState;
import frc.robot.commands.collectorCommands.singleCollectorCommands.SingleCollectorDown;
import frc.robot.commands.collectorCommands.singleCollectorCommands.SingleCollectorIntake;
import frc.robot.commands.hopperCommands.HopperVelocityCommand;
import frc.robot.commands.indexerCommands.IndexerAutoUnjam;
import frc.robot.commands.indexerCommands.IndexerReverseCommand;
import frc.robot.subsystems.collector.collectorConstants.CollectorTalonLeftConstants;
import frc.robot.subsystems.collector.collectorConstants.CollectorTalonRightConstants;
import frc.robot.subsystems.collector.collectorConstants.CollectorTalonSingleConstants;
import frc.robot.subsystems.collector.doubleCollector.CollectorModuleIO;
import frc.robot.subsystems.collector.doubleCollector.CollectorModuleSimIO;
import frc.robot.subsystems.collector.doubleCollector.CollectorModuleTalonIO;
import frc.robot.subsystems.collector.doubleCollector.DoubleCollector;
import frc.robot.subsystems.collector.singleCollector.CollectorModuleSingleIO;
import frc.robot.subsystems.collector.singleCollector.CollectorModuleSingleTalonIO;
import frc.robot.subsystems.collector.singleCollector.SingleCollector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.HopperIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.shooter.BeamBreakIO;
import frc.robot.subsystems.shooter.BeamBreakIOSensor;
import frc.robot.subsystems.shooter.FlywheelIO;
import frc.robot.subsystems.shooter.FlywheelIOSim;
import frc.robot.subsystems.shooter.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.PivotIO;
import frc.robot.subsystems.shooter.PivotIOSim;
import frc.robot.subsystems.shooter.PivotIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelLevel;
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
  private final Shooter shooter;
  private final Indexer indexer;
  private final DoubleCollector doubleCollector;
  private final SingleCollector singleCollector;
  private final Hopper hopper;

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

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Sim
  private SwerveDriveSimulation simulation = null;
  private Pose2d robotSimStartPosBlue = new Pose2d(7.485, 7.064, new Rotation2d(Math.PI));
  private Pose2d robotSimStartPosRed = new Pose2d(10, 1.5, new Rotation2d());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    leftJoystick = new Joystick(Constants.OIConstants.LeftJoystick);
    rightJoystick = new Joystick(Constants.OIConstants.RightJoystick);
    buttonJoystick = new Joystick(Constants.OIConstants.ButtonJoystick);

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
                (pose) -> {});

        shooter =
            new Shooter(
                new FlywheelIOTalonFX(ShooterConstants.upperFlywheelMotorID, FlywheelLevel.Upper),
                new FlywheelIOTalonFX(ShooterConstants.lowerFlywheelMotorID, FlywheelLevel.Lower),
                new PivotIOTalonFX(),
                new BeamBreakIOSensor());

        vision =
            new Vision(
                drive,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));

        // This is a solution to the vision-drive cross dependency issue that hurts my
        // eys, but it does work

        indexer = new Indexer(new IndexerIOTalonFX());

        doubleCollector =
            new DoubleCollector(
                new CollectorModuleTalonIO(new CollectorTalonLeftConstants()),
                new CollectorModuleTalonIO(new CollectorTalonRightConstants()));

        singleCollector = new SingleCollector(new CollectorModuleSingleIO() {});

        hopper = new Hopper(new HopperIO() {});

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
                simulation::setSimulationWorldPose);

        shooter =
            new Shooter(
                new FlywheelIOSim(ShooterConstants.upperFlywheelMotorID, FlywheelLevel.Upper),
                new FlywheelIOSim(ShooterConstants.lowerFlywheelMotorID, FlywheelLevel.Lower),
                new PivotIOSim(),
                new BeamBreakIO() {});

        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name,
                    VisionConstants.robotToCamera0,
                    simulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name,
                    VisionConstants.robotToCamera1,
                    simulation::getSimulatedDriveTrainPose));

        doubleCollector =
            new DoubleCollector(
                new CollectorModuleSimIO(simulation, new CollectorTalonLeftConstants()),
                new CollectorModuleSimIO(simulation, new CollectorTalonRightConstants()));

        singleCollector = new SingleCollector(new CollectorModuleSingleIO() {});

        indexer = new Indexer(new IndexerIOSim());

        hopper = new Hopper(new HopperIOSim());

        break;

      case SINGLEREAL:
        // Real robot with single collector, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(Constants.tunerConstants.getFrontLeft()),
                new ModuleIOTalonFX(Constants.tunerConstants.getFrontRight()),
                new ModuleIOTalonFX(Constants.tunerConstants.getBackLeft()),
                new ModuleIOTalonFX(Constants.tunerConstants.getBackRight()),
                (pose) -> {});

        shooter =
            new Shooter(
                new FlywheelIOTalonFX(ShooterConstants.upperFlywheelMotorID, FlywheelLevel.Upper),
                new FlywheelIOTalonFX(ShooterConstants.lowerFlywheelMotorID, FlywheelLevel.Lower),
                new PivotIOTalonFX(),
                new BeamBreakIOSensor());

        vision =
            new Vision(
                drive, new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));

        indexer = new Indexer(new IndexerIOTalonFX());

        doubleCollector =
            new DoubleCollector(new CollectorModuleIO() {}, new CollectorModuleIO() {});

        singleCollector =
            new SingleCollector(
                new CollectorModuleSingleTalonIO(new CollectorTalonSingleConstants()));

        hopper = new Hopper(new HopperIOTalonFX());

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
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
                (pose) -> {});
        shooter =
            new Shooter(
                new FlywheelIO() {}, new FlywheelIO() {}, new PivotIO() {}, new BeamBreakIO() {});

        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});

        doubleCollector =
            new DoubleCollector(new CollectorModuleIO() {}, new CollectorModuleIO() {});
        singleCollector = new SingleCollector(new CollectorModuleSingleIO() {});

        indexer = new Indexer(new IndexerIO() {});

        hopper = new Hopper(new HopperIOTalonFX() {});

        break;
    }
    Command autoShootIndexerCommand = new IndexerAutoUnjam(indexer, shooter::getBeamBreak, hopper);

    NamedCommands.registerCommand(
        "Shoot Preset Setpoits ", new ShootPresetSetpoints(shooter, TargetType.GOAL));
    NamedCommands.registerCommand("Both Collectors Up", new BothCollectorsUpState(doubleCollector));
    NamedCommands.registerCommand(
        "Auto Aimed Shoot", new AutonomousAimedShoot(drive, shooter, indexer));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // Set up SysId routines
    autoChooserAddOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooserAddOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));

    autoChooserAddOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooserAddOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooserAddOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooserAddOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooserAddOption(
        "Shooter SysId (Quasistatic Forward)",
        shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooserAddOption(
        "Shooter SysId (Quasistatic Reverse)",
        shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooserAddOption(
        "Shooter SysId (Dynamic Forward)", shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooserAddOption(
        "Shooter SysId (Dynamic Reverse)", shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption("Simple Auto", new NoYawShootHub(shooter, drive));

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
    Trigger shootAutoAim;
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -leftJoystick.getY(), // -X (used to be)
            () -> -leftJoystick.getX(), // -Y (used to be)
            () -> -rightJoystick.getX())); // Confirm if negitive or not

    shootAutoAim = new JoystickButton(rightJoystick, 1);

    shootAutoAim.whileTrue(
        new DriveAimedAutoShoot(
            drive, shooter, () -> -leftJoystick.getY(), () -> -leftJoystick.getX()));

    ShootPresetSetpoints shootPresetSetpointsGoal =
        new ShootPresetSetpoints(shooter, TargetType.GOAL);
    ShootPresetSetpoints shootPresetSetpointsTransfer =
        new ShootPresetSetpoints(shooter, TargetType.TRANSFER);

    JoystickButton runIndexer = new JoystickButton(leftJoystick, 1);
    JoystickButton runIndexerReverse =
        new JoystickButton(buttonJoystick, Constants.OIConstants.CollectorReverseIntakeID);

    runIndexer.whileTrue(new IndexerAutoUnjam(indexer, shooter::getBeamBreak, hopper));

    runIndexerReverse.whileTrue(
        new IndexerReverseCommand(indexer)
            .alongWith(
                new HopperVelocityCommand(
                    hopper,
                    RotationsPerSecond.of(
                        HopperConstants.hopperReverseAngularVelocityRotationsPerSecond.get()))));

    // if the index button is hed and more than a second elapses before the beam
    // break is broken the
    // indexer will reverse for 1 second before attempting to intake again

    Logger.recordOutput("Control Scheme", Constants.controlScheme);

    TestPathCommands.publishTestPaths(drive);

    shooter.setDefaultCommand(Commands.run(shooter::stop, shooter));

    switch (Constants.currentMode) {
      case REAL:
        // State Buttons
        JoystickButton manualModeSwitch =
            new JoystickButton(buttonJoystick, Constants.OIConstants.CollectorManualModeID);
        JoystickButton autoModeSwitch =
            new JoystickButton(buttonJoystick, Constants.OIConstants.CollectorAutoModeID);

        manualModeSwitch
            .negate()
            .and(autoModeSwitch.negate())
            .onTrue(new BothCollectorsUpState(doubleCollector));

        // Collector Intake Button
        new JoystickButton(buttonJoystick, Constants.OIConstants.CollectorIntakeID)
            .whileTrue(new CollectorIntake(doubleCollector, indexer));

        // Collector Reverse Intake Button
        new JoystickButton(buttonJoystick, Constants.OIConstants.CollectorReverseIntakeID)
            .whileTrue(new CollectorExpel(doubleCollector));

        // Right Collector Down
        new JoystickButton(buttonJoystick, Constants.OIConstants.CollectorRightDownID)
            .and(manualModeSwitch)
            .onTrue(new RightCollectorDownState(doubleCollector));

        // Left Collector Down
        new JoystickButton(buttonJoystick, Constants.OIConstants.CollectorLeftDownID)
            // .and(manualModeSwitch)
            .onTrue(new LeftCollectorDownState(doubleCollector));

        // Both Collectors Up
        new JoystickButton(buttonJoystick, Constants.OIConstants.CollectorManualUpID)
            // .and(manualModeSwitch)
            .onTrue(new BothCollectorsUpState(doubleCollector));

        new JoystickButton(buttonJoystick, Constants.OIConstants.CollectorManualPivotDown)
            // .and(manualModeSwitch)
            .whileTrue(new ManualCollectorDown(doubleCollector));

        new JoystickButton(buttonJoystick, Constants.OIConstants.CollectorManualPivotUp)
            // .and(manualModeSwitch)
            .whileTrue(new ManualCollectorUp(doubleCollector));

        new JoystickButton(buttonJoystick, Constants.OIConstants.ShooterShootTransferID)
            .whileTrue(shootPresetSetpointsTransfer);

        new JoystickButton(buttonJoystick, Constants.OIConstants.ShooterShootHubID)
            .whileTrue(shootPresetSetpointsGoal);
        break;

      case SINGLEREAL:
        new JoystickButton(buttonJoystick, Constants.OIConstants.CollectorRightDownID)
            .onTrue(new SingleCollectorDown(singleCollector));
        new JoystickButton(buttonJoystick, Constants.OIConstants.CollectorIntakeID)
            .whileTrue(new SingleCollectorIntake(singleCollector));
        break;

      default:
        break;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get().alongWith(new HoldPosition(doubleCollector));
  }

  /**
   * Use this to pass the teleop beggining command to the main {@link Robot} class.
   *
   * @return the command to run in teleop init
   */
  public Command getTeleopInitCommand() {
    return new BothCollectorsUpState(doubleCollector);
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
      simulation.setSimulationWorldPose(robotSimStartPosBlue);
    } else {
      simulation.setSimulationWorldPose(robotSimStartPosRed);
    }
  }

  /** Creates a new simulated fuel and launches it from the current robot position and speeds */
  public void launchSimFuel() {
    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            SimProjectile.getProjectile(
                simulation.getSimulatedDriveTrainPose(),
                simulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                shooter.getShooterForwardLinearVelocity(),
                shooter.getPivotAngle()));
  }

  /** auto chooser option that dosent crash robot when ran slong side holdPosition */
  public void autoChooserAddOption(String key, Command command) {
    autoChooser.addOption(key, command);
  }

  public void scheduleBothCollectorsUp() {
    CommandScheduler.getInstance().schedule(new BothCollectorsUpState(doubleCollector));
  }
}
