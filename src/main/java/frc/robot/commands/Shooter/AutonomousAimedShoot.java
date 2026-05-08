package frc.robot.commands.Shooter;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveControlConstants;
import frc.robot.commands.DriveUtils;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.TriggerConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterSetpoints;
import frc.robot.util.ShooterCalculations;
import frc.robot.util.ShooterCalculations.ShooterControls;
import org.littletonrobotics.junction.Logger;

public class AutonomousAimedShoot extends Command {

  private final Drive drive;
  private final Shooter shooter;
  private final Command feedCommand;

  private final ProfiledPIDController yawController;

  private final Debouncer triggerDebounce = new Debouncer(0.5);

  /**
   * Auto-aiming shoot command specifically for auto. It does not allow for driver translation of
   * the robot and it automatically starts shooting when setpoints are reached
   *
   * @param feedCommand The command that should be run to feed balls into the shooter
   */
  public AutonomousAimedShoot(Drive drive, Shooter shooter, Command feedCommand) {
    this.drive = drive;
    this.shooter = shooter;
    this.feedCommand = feedCommand;
    yawController =
        new ProfiledPIDController(
            DriveControlConstants.YAW_CONTROL_KP.get(),
            0.0,
            DriveControlConstants.YAW_CONTROL_KD.get(),
            new Constraints(
                DriveControlConstants.YAW_MAX_VELOCITY.get(),
                DriveControlConstants.YAW_MAX_ACCELERATION.get()));
    yawController.enableContinuousInput(-Math.PI, Math.PI);
    yawController.setTolerance(DriveControlConstants.YAW_SETPOINT_TOLERANCE_RADIANS.get());

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    yawController.reset(drive.getRotation().getRadians());
    hopper.setTriggerDutyCycle(0.0);
  }

  @Override
  public void execute() {
    var currentPose = drive.getPose();
    var currentSpeeds = drive.getChassisSpeeds();

    ShooterControls controls =
        ShooterCalculations.getApexConstrainedShooterControls(currentPose, currentSpeeds);

    shooter.setShooterSetpoints(
        new ShooterSetpoints(controls.flywheelVelocity(), controls.shooterPitch()));
    double currentYawRadians = drive.getRotation().getRadians();
    double targetYawRadians = controls.driveYaw().getRadians();

    Logger.recordOutput("Auto Shoot/Target Yaw", targetYawRadians);
    Logger.recordOutput("Auto Shoot/Current Yaw", currentYawRadians);
    double omegaCalculated;

    omegaCalculated = yawController.calculate(currentYawRadians, targetYawRadians);
    Logger.recordOutput("Auto Shoot/Omega Calculated", omegaCalculated);

    ChassisSpeeds targetSpeeds =
        DriveUtils.getChassisSpeeds(
            drive, () -> 0, () -> 0, omegaCalculated // Override rotation
            );

    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    Rotation2d fieldCentricOffset = isRed ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            targetSpeeds, drive.getRotation().plus(fieldCentricOffset)));

    Logger.recordOutput("Auto Shoot/At Setpoint", yawController.atSetpoint());
    Logger.recordOutput("Auto Shoot/At Goal", yawController.atGoal());
    Logger.recordOutput(
        "Auto Shoot/Debounced", triggerDebounce.calculate(yawController.atSetpoint()));
        "Auto Shoot/Debounced", triggerDebounce.calculate(yawController.atSetpoint()));

    boolean readyToFeed = yawController.atGoal() && shooter.isFlywheelAtSetpoint();
    Logger.recordOutput("Auto Shoot/Trigger Ready", readyToFeed);

    hopper.setTriggerVelocity(RotationsPerSecond.of(TriggerConstants.triggerAngularVelocityRotationsPerSecond.get()));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopFlywheels();
    hopper.setTriggerDutyCycle(0);
  }
}
