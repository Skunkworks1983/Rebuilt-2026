package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
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

public class DriveTransfer extends Command {
  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final PIDController yawController;
  private final SimpleShooter shooter;

  private double lastFilteredYaw;
  private LinearFilter yawFilter;

  private final LoggedTunableNumber yawKP =
      new LoggedTunableNumber("Auto Aim/Yaw KP", DriveControlConstants.yawControlKp);
  private final LoggedTunableNumber yawKD =
      new LoggedTunableNumber("Auto Aim/Yaw KD", DriveControlConstants.yawControlKD);
  private final LoggedTunableNumber yawTolerance =
      new LoggedTunableNumber("Auto Aim/Yaw Tolerance", DriveControlConstants.yawSetpointTolerance);
  private final LoggedTunableNumber linearFilterTimeConstant =
      new LoggedTunableNumber(
          "Auto Aim/Linear Filter Time Constant", DriveControlConstants.linearFilterTimeConstant);
  private final LoggedTunableNumber linearFilterPeriod =
      new LoggedTunableNumber(
          "Auto Aim/Linear Filter Period", DriveControlConstants.linearFilterPeriod);

  public DriveTransfer(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, SimpleShooter shooter) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.shooter = shooter;

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
    lastFilteredYaw = drive.getRotation().getRadians();
    yawFilter =
        LinearFilter.singlePoleIIR(linearFilterTimeConstant.get(), linearFilterPeriod.get());
    yawFilter.reset();
  }

  @Override
  public void execute() {
    var currentPose = drive.getPose();
    var currentSpeeds = drive.getChassisSpeeds(); // Robot Relative

    NoPitchShooterControls controls =
        ShooterCalculations.getShooterControlsTransfer(currentSpeeds, currentPose);

    shooter.setShooterControls(controls.upperFlywheelVelocity(), controls.lowerFlywheelVelocity());
    double currentYawRadians = drive.getRotation().getRadians();
    double targetYawRadians = controls.driveYaw().getRadians();
    double filteredTargetYaw;

    // if (Math.abs(currentYawRadians - targetYawRadians) >
    // DriveControlConstants.largeAngleJumpThreshold.in(Radians)) {
    //   filteredTargetYaw = targetYawRadians;
    //   lastFilteredYaw = targetYawRadians;
    //   yawFilter.reset();
    // } else {
    //   double delta = MathUtil.angleModulus(targetYawRadians - lastFilteredYaw);
    //   double unwrappedTarget = currentYawRadians + delta;
    //   double filtered = yawFilter.calculate(unwrappedTarget);
    //   lastFilteredYaw = filtered;

    //   filteredTargetYaw = MathUtil.angleModulus(filtered);
    // }

    Logger.recordOutput("Auto Shoot/Target Yaw", targetYawRadians);
    Logger.recordOutput("Auto Shoot/Current Yaw", currentYawRadians);
    Logger.recordOutput("Auto Shoot/Filtered Target Yaw", targetYawRadians);

    Logger.recordOutput("Auto Shoot/Yaw Error", targetYawRadians - currentYawRadians);
    Logger.recordOutput(
        "Auto Shoot/Upper Flywheel Error",
        controls
            .upperFlywheelVelocity()
            .minus(shooter.getUpperFlywheelVelocity())
            .in(RotationsPerSecond),
        RotationsPerSecond);
    Logger.recordOutput(
        "Auto Shoot/Lower Flywheel Error",
        controls
            .lowerFlywheelVelocity()
            .minus(shooter.getLowerFlywheelVelocity())
            .in(RotationsPerSecond),
        RotationsPerSecond);

    double omegaCmd = yawController.calculate(currentYawRadians, targetYawRadians);

    ChassisSpeeds targetSpeeds =
        DriveUtils.getChassisSpeeds(
            drive, xSupplier, ySupplier, omegaCmd // Override rotation
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
    shooter.setShooterControls(RadiansPerSecond.of(0), DegreesPerSecond.of(0));
  }
}
