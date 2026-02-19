package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.simpleShooter.ShooterConstants;
import frc.robot.subsystems.simpleShooter.SimpleShooter;
import frc.robot.util.ShooterCalculations;
import frc.robot.util.ShooterCalculations.NoPitchShooterControls;
import frc.robot.util.ShooterCalculations.ShooterControls;
import org.littletonrobotics.junction.Logger;

public class NoDriveShootHub extends Command {
  private final SimpleShooter shooter;
  private final Drive drive;

  /**
   * Sets shooter to correct flywheel velocity and pivot pitch to shoot into the hub without
   * controlling drive yaw
   *
   * <p>This is primarily for autonomous
   *
   * @param shooter The shooter..
   * @param drive The drivetrain....
   */
  public NoDriveShootHub(SimpleShooter shooter, Drive drive) {
    this.shooter = shooter;
    this.drive = drive;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Commands/Auto Shoot/Drive Aimed Goal No Yaw", true);
  }

  @Override
  public void execute() {
    NoPitchShooterControls controls =
        ShooterCalculations.getNoPitchExtrapolatedControls(
            drive.getPose(), drive.getChassisSpeeds(), ShooterConstants.shooterPitch);

    shooter.setShooterControls(controls.upperFlywheelVelocity(), controls.lowerFlywheelVelocity());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setShooterControls(RadiansPerSecond.of(0), RadiansPerSecond.of(0));
    Logger.recordOutput("Commands/Auto Shoot/Drive Aimed Goal No Yaw", false);
  }
}