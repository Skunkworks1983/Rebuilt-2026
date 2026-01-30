package frc.robot.util.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.shooter.ShooterConstants;
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

    return new RebuiltFuelOnFly(
            drivePoseFieldRelative.getTranslation(),
            new Translation2d(),
            fieldRelativeSpeeds,
            drivePoseFieldRelative.getRotation(),
            ShooterConstants.shooterInitalPosition.getMeasureZ(),
            launchVelocity,
            shooterAngle)
        .withProjectileTrajectoryDisplayCallBack(
            // Callback for when the note will eventually hit the target (no target configured)
            (pose3ds) ->
                Logger.recordOutput(
                    "Shooter/FuelProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
            (pose3ds) ->
                Logger.recordOutput(
                    "Shooter/FuelProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new)));
  }
}
