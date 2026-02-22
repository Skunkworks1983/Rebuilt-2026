package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.simpleShooter.SimpleShooter;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ShooterCalculations;
import frc.robot.util.ShooterCalculations.NoPitchShooterControls;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveAimedTransfer extends Command {

  private final Drive drive;
  private final SimpleShooter shooter;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final PIDController yawController;

  private final LoggedTunableNumber yawKP =
      new LoggedTunableNumber("DriveAimedTransfer/Yaw KP", DriveControlConstants.yawControlKp);
  private final LoggedTunableNumber yawKD =
      new LoggedTunableNumber("DriveAimedTransfer/Yaw KD", DriveControlConstants.yawControlKD);
  private final LoggedTunableNumber yawTolerance =
      new LoggedTunableNumber(
          "DriveAimedTransfer/Yaw Tolerance", DriveControlConstants.yawSetpointTolerance);

  public DriveAimedTransfer(
      Drive drive, SimpleShooter shooter, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    this.drive = drive;
    this.shooter = shooter;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    yawController =
        new PIDController(
            DriveControlConstants.yawControlKp, 0.0, DriveControlConstants.yawControlKD);
    yawController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive, shooter);
  }

  @Override
  public void initialize() {
    yawController.reset();
    yawController.setP(yawKP.get());
    yawController.setD(yawKD.get());
    yawController.setTolerance(yawTolerance.get());
  }

  @Override
  public void execute() {
    var currentPose = drive.getPose();
    var currentSpeeds = drive.getChassisSpeeds();

    NoPitchShooterControls controls =
        ShooterCalculations.getShooterControlsTransfer(currentSpeeds, currentPose);

    shooter.setShooterControls(controls.upperFlywheelVelocity(), controls.lowerFlywheelVelocity());

    double currentYawRadians = drive.getRotation().getRadians();
    double targetYawRadians = controls.driveYaw().getRadians();

    Logger.recordOutput("DriveAimedTransfer/Target Yaw", targetYawRadians);
    Logger.recordOutput("DriveAimedTransfer/Current Yaw", currentYawRadians);

    double omegaCalculated = yawController.calculate(currentYawRadians, targetYawRadians);

    ChassisSpeeds targetSpeeds =
        DriveUtils.getChassisSpeeds(
            drive, xSupplier, ySupplier, omegaCalculated // Override rotation
            );

    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    Rotation2d fieldCentricOffset = isRed ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            targetSpeeds, drive.getRotation().plus(fieldCentricOffset)));
  }

  @Override
  public void end(boolean interrupted) {
    yawController.close();
  }
}
