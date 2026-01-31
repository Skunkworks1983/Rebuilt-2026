package frc.robot.util.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.FieldConstants;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class SimProjectile {
  public static GamePieceProjectile getProjectile(
      Pose2d drivePoseFieldRelative,
      ChassisSpeeds fieldRelativeSpeeds,
      LinearVelocity launchVelocity,
      Angle shooterAngle) {
    Logger.recordOutput("Shooter/SimLinearLaunchVelocity", launchVelocity);
    Logger.recordOutput("Shooter/SimFieldRelativeRobot", drivePoseFieldRelative);

    Translation3d targetPosition =
        DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Blue
            ? FieldConstants.blueGoal
            : FieldConstants.redGoal;

    return new RebuiltFuelOnFly(
            drivePoseFieldRelative.getTranslation(),
            new Translation2d(),
            fieldRelativeSpeeds,
            drivePoseFieldRelative.getRotation(),
            ShooterConstants.shooterInitalPosition.getMeasureZ(),
            launchVelocity,
            shooterAngle)
        .withTargetPosition(() -> targetPosition)
        .withProjectileTrajectoryDisplayCallBack(
            (pose3ds) ->
                Logger.recordOutput(
                    "Shooter/FuelProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
            (pose3ds) ->
                Logger.recordOutput(
                    "Shooter/FuelProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new)));
  }
}
