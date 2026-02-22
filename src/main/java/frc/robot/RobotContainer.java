// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.events.PointTowardsZoneTrigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveAimedShoot;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.Shooter.NoYawShootHub;
import frc.robot.commands.TestPathCommands;
import frc.robot.commands.collectorCommands.AutomaticStateChange;
import frc.robot.commands.collectorCommands.BothCollectorsUpState;
import frc.robot.commands.collectorCommands.CollectorIntake;
import frc.robot.commands.collectorCommands.LeftCollectorDownState;
import frc.robot.commands.collectorCommands.RightCollectorDownState;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.collector.CollectorModuleSimIO;
import frc.robot.subsystems.collector.CollectorModuleTalonIO;
import frc.robot.subsystems.collector.collectorConstants.CollectorTalonLeftConstants;
import frc.robot.subsystems.collector.collectorConstants.CollectorTalonRightConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.shooter.FlywheelIO;
import frc.robot.subsystems.shooter.FlywheelIOSim;
import frc.robot.subsystems.shooter.PivotIO;
import frc.robot.subsystems.shooter.PivotIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
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
  private final Collector collector;
  private final Indexer indexer;

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

  // Sim
  private SwerveDriveSimulation simulation = null;
  private Pose2d robotSimStartPosBlue = new Pose2d(7.485, 7.064, new Rotation2d(Math.PI));
  private Pose2d robotSimStartPosRed = new Pose2d(10, 1.5, new Rotation2d());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.controlScheme == Constants.ControlScheme.OI) {
      leftJoystick = new Joystick(Constants.JoystickConstants.LeftJoystick);
      rightJoystick = new Joystick(Constants.JoystickConstants.RightJoystick);
      buttonJoystick = new Joystick(Constants.JoystickConstants.ButtonJoystick);
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
        // ModuleIOTalonFX is intended for modules  ith TalonFX drive, TalonFX turn, and
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

        shooter = new Shooter(new FlywheelIO() {}, new FlywheelIO() {}, new PivotIO() {});

        vision =
            new Vision(
                drive, new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));

        // This is a solution to the vision-drive cross dependency issue that hurts my
        // eys, but it does work
        drive.setResetVisionIMUCallback((pose) -> vision.resetCameraIMU(pose));

        indexer = new Indexer(new IndexerIOTalonFX(IndexerConstants.indexerMotorID));

        collector =
            new Collector(
                new CollectorModuleTalonIO(new CollectorTalonLeftConstants()),
                new CollectorModuleTalonIO(new CollectorTalonRightConstants()));

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
                simulation::setSimulationWorldPose,
                (pose) -> {});

        shooter =
            new Shooter(
                new FlywheelIOSim(ShooterConstants.flywheelMotorOneId),
                new FlywheelIOSim(ShooterConstants.flywheelSecondaryMotorId),
                new PivotIOSim());

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

        collector =
            new Collector(
                new CollectorModuleSimIO(simulation, new CollectorTalonLeftConstants()),
                new CollectorModuleSimIO(simulation, new CollectorTalonRightConstants()));

        indexer = new Indexer(new IndexerIOSim(IndexerConstants.indexerMotorID));

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
        shooter = new Shooter(new FlywheelIO() {}, new FlywheelIO() {}, new PivotIO() {});

        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});

        collector =
            new Collector(
                new CollectorModuleSimIO(simulation, new CollectorTalonLeftConstants()),
                new CollectorModuleSimIO(simulation, new CollectorTalonRightConstants()));

        indexer = new Indexer(new IndexerIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    new EventTrigger("FIRE")
        .onTrue(Commands.print("LEBRON!").andThen(new InstantCommand(this::launchSimFuel)));
    new PointTowardsZoneTrigger("Aim Hub").whileTrue(new NoYawShootHub(shooter, drive));

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

    autoChooser.addOption(
        "Shooter SysId (Quasistatic Forward)",
        shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Shooter SysId (Quasistatic Reverse)",
        shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Shooter SysId (Dynamic Forward)", shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Shooter SysId (Dynamic Reverse)", shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
    Trigger shootGoalButton;
    Trigger shootTransferButton;
    JoystickButton runIndexer;
    // Default command, normal field-relative drive
    if (Constants.controlScheme == Constants.ControlScheme.OI) {
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -leftJoystick.getY(), // -X (used to be)
              () -> -leftJoystick.getX(), // -Y (used to be)
              () -> rightJoystick.getX())); // Confirm if negitive or not

      shootGoalButton = new JoystickButton(leftJoystick, 1);
      Trigger shootButton = new JoystickButton(rightJoystick, 1);

      shootGoalButton.whileTrue(
          new DriveAimedShoot(
              drive,
              shooter,
              () -> -leftJoystick.getY(),
              () -> -leftJoystick.getX(),
              DriveAimedShoot.TargetType.GOAL));
      shootButton.onTrue(new InstantCommand(this::launchSimFuel));

      runIndexer = new JoystickButton(buttonJoystick, IndexerConstants.runIndexerButtonID);

    } else {
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -xboxController.getLeftY(),
              () -> -xboxController.getLeftX(),
              () -> -xboxController.getRightX()));
      shootGoalButton = new JoystickButton(xboxController, XboxController.Button.kA.value);
      shootTransferButton = new JoystickButton(xboxController, XboxController.Button.kY.value);
      Trigger shootButton = new JoystickButton(xboxController, XboxController.Button.kB.value);

      shootGoalButton.whileTrue(
          new DriveAimedShoot(
              drive,
              shooter,
              () -> -xboxController.getLeftY(),
              () -> -xboxController.getLeftX(),
              DriveAimedShoot.TargetType.GOAL));
      shootTransferButton.whileTrue(
          new DriveAimedShoot(
              drive,
              shooter,
              () -> -xboxController.getLeftY(),
              () -> -xboxController.getLeftX(),
              DriveAimedShoot.TargetType.TRANSFER));
      shootButton.onTrue(new InstantCommand(this::launchSimFuel));
      runIndexer = new JoystickButton(xboxController, IndexerConstants.runIndexerButtonID);
    }

    runIndexer.whileTrue(new IndexerCommand(indexer));
    Logger.recordOutput("Control Scheme", Constants.controlScheme);

    TestPathCommands.publishTestPaths(drive);

    SmartDashboard.putData("Shooter/Sim Shoot", new InstantCommand(this::launchSimFuel));
    SmartDashboard.putData(
        "Shooter/Set Manual Velocity",
        new InstantCommand(() -> shooter.setManualVelocity(RotationsPerSecond.of(25))));

    // State Buttons
    JoystickButton manualModeSwitch =
        new JoystickButton(buttonJoystick, Constants.JoystickConstants.CollectorManualModeID);
    JoystickButton autoModeSwitch =
        new JoystickButton(buttonJoystick, Constants.JoystickConstants.CollectorAutoModeID);

    autoModeSwitch.onTrue(new AutomaticStateChange(collector, () -> drive.getDirection()));

    manualModeSwitch
        .negate()
        .and(autoModeSwitch.negate())
        .onTrue(new BothCollectorsUpState(collector));

    // Collector Intake Button
    new JoystickButton(buttonJoystick, Constants.JoystickConstants.CollectorIntakeID)
        .whileTrue(new CollectorIntake(collector));

    // Right Collector Down
    new JoystickButton(buttonJoystick, Constants.JoystickConstants.CollectorRightDownID)
        .and(manualModeSwitch)
        .onTrue(new RightCollectorDownState(collector));

    // Left Collector Down
    new JoystickButton(buttonJoystick, Constants.JoystickConstants.CollectorLeftDownID)
        .and(manualModeSwitch)
        .onTrue(new LeftCollectorDownState(collector));

    // Both Collectors Up
    new JoystickButton(leftJoystick, Constants.JoystickConstants.CollectorBothUpID)
        .and(manualModeSwitch)
        .onTrue(new BothCollectorsUpState(collector));

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
}
