package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.shooter.ShooterConstants;

public class ShooterCalculations {
  public record ShooterControls(
      AngularVelocity shooterVelocity, Angle shooterPitch, Rotation2d driveYaw) {}
  ;

  public static ShooterControls getShooterControlsGoal(
      ChassisSpeeds speeds, Pose2d odometryPose, ShooterInterporlations shooterInterporlations) {
    Translation3d target =
        DriverStation.getAlliance().get() == Alliance.Red
            ? FieldConstants.redGoal
            : FieldConstants.blueGoal;

    Translation2d targetVector = target.toTranslation2d().minus(odometryPose.getTranslation());

    Distance distance = Meters.of(targetVector.getNorm());

    AngularVelocity shooterVelocity = shooterInterporlations.getShooterVelocityGoal(distance);
    Angle shooterPitch = shooterInterporlations.getPivotAngleGoal(distance);
    Rotation2d rawYaw = targetVector.getAngle();

    // Get field relative chassis speeds
    Translation2d robotVel =
        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
            .rotateBy(odometryPose.getRotation());
    double fieldVx = robotVel.getX();
    double fieldVy = robotVel.getY();

    // Unit vectors of the shot direction (field frame)
    double ux = rawYaw.getCos();
    double uy = rawYaw.getSin();

    // Components of the robot velocity along the shot-forward and shot-lateral
    // directions
    double parallelComponent = fieldVx * ux + fieldVy * uy; // Velocity along direction of shot
    double perpendicularComponent = fieldVx * -uy + fieldVy * ux; // Velocity perpendicular to shot

    // Flywheel parallel velocity
    double shooterLinear =
        shooterVelocity.in(RadiansPerSecond)
            * ShooterConstants.flywheelRadius.in(Meters)
            * Math.cos(shooterPitch.in(Radians));

    Angle compensationAngle =
        Radians.of(Math.atan2(perpendicularComponent, shooterLinear + parallelComponent));

    return new ShooterControls(
        shooterVelocity, shooterPitch, rawYaw.minus(new Rotation2d(compensationAngle)));
  }

  public static ShooterControls getOptimizedShooterControls(
      Pose2d odometryPose,
      ChassisSpeeds chassisSpeeds,
      Angle prevShooterPitch,
      AngularVelocity prevFlywheelVelocity) {
    Translation3d target =
        DriverStation.getAlliance().get() == Alliance.Red
            ? FieldConstants.redGoal
            : FieldConstants.blueGoal;

    Translation2d targetVector = target.toTranslation2d().minus(odometryPose.getTranslation());

    // Intentionally move everything to doubles for math in iterative steps
    double x = targetVector.getNorm();
    double z = target.getZ() - ShooterConstants.shooterInitalPosition.getZ();

    Rotation2d rawYaw = targetVector.getAngle();
    double shotAngleRadians = rawYaw.getRadians();

    double velocityRobotAlongShot =
        chassisSpeeds.vxMetersPerSecond * Math.cos(shotAngleRadians)
            + chassisSpeeds.vyMetersPerSecond * Math.sin(shotAngleRadians);

    double shooterPitch = prevShooterPitch.in(Radians);
    double flywheelLinearVelocity =
        prevFlywheelVelocity.in(RadiansPerSecond) * ShooterConstants.flywheelRadius.in(Meters);

    for (int i = 0; i < ShooterSolverConstants.maxIterations; i++) {
      // Trig once for optimization
      double cosTheta = Math.cos(shooterPitch);
      double sinTheta = Math.sin(shooterPitch);

      // Velocity of flywheel + robot velocity in flywheel direction
      double effectiveVelocity = flywheelLinearVelocity * cosTheta + velocityRobotAlongShot;

      // Error of current pitch and velocity from target
      double e =
          x * Math.tan(shooterPitch)
              - (0.5 * ShooterSolverConstants.gravity * x * x)
                  / (effectiveVelocity * effectiveVelocity)
              - z;

      if (Math.abs(e) < ShooterSolverConstants.maxError) break;

      // Partial derivative Pitch (theta)
      double de_dtheta =
          (x / (cosTheta * cosTheta))
              + (ShooterSolverConstants.gravity * x * x * (-flywheelLinearVelocity * sinTheta))
                  / Math.pow(effectiveVelocity, 3);

      // Partial derivative Velocity
      double de_dv =
          (ShooterSolverConstants.gravity * x * x * cosTheta) / Math.pow(effectiveVelocity, 3);

      // Update step using Gradient Descent
      double denominator = (de_dtheta * de_dtheta + de_dv * de_dv);

      // Safety epsilon to prevent NaN
      double lambda = e / (denominator + 1e-6);

      shooterPitch -= lambda * de_dtheta * ShooterSolverConstants.weightTheta;
      flywheelLinearVelocity -= lambda * de_dv * ShooterSolverConstants.weightVelocity;

      shooterPitch =
          MathUtil.clamp(
              shooterPitch,
              ShooterConstants.minShooterPitch.in(Radians),
              ShooterConstants.maxShooterPitch.in(Radians));
    }

    // Get field relative chassis speeds
    Translation2d fieldRelativeRobotVel =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .rotateBy(odometryPose.getRotation());

    // Unit vectors of the shot direction (do once to minimize trig)
    double ux = rawYaw.getCos();
    double uy = rawYaw.getSin();

    double parallelComponent =
        fieldRelativeRobotVel.getX() * ux + fieldRelativeRobotVel.getY() * uy;

    double perpendicularComponent =
        fieldRelativeRobotVel.getX() * -uy + fieldRelativeRobotVel.getY() * ux;

    Angle compensationAngle =
        Radians.of(Math.atan2(perpendicularComponent, flywheelLinearVelocity + parallelComponent));

    return new ShooterControls(
        RadiansPerSecond.of(flywheelLinearVelocity / ShooterConstants.flywheelRadius.in(Meters)),
        Radians.of(shooterPitch),
        rawYaw.minus(new Rotation2d(compensationAngle)));
  }

  public static ShooterControls getShooterControlsTransfer(
      ChassisSpeeds speeds, Pose2d odometryPose, ShooterInterporlations shooterInterporlations) {
    Translation2d target =
        DriverStation.getAlliance().get() == Alliance.Red
            ? FieldConstants.redFuelTransferTarget
            : FieldConstants.blueFuelTransferTarget;
    // TODO: ONLY ALLOW LEGAL ANGLES (DOESNT HIT TOWER)
    Translation2d targetVector = target.minus(odometryPose.getTranslation());

    Distance distance = Meters.of(targetVector.getNorm());

    AngularVelocity shooterVelocity = shooterInterporlations.getShooterVelocityTransfer(distance);
    Angle shooterPitch = shooterInterporlations.getPivotAngleTransfer(distance);
    Rotation2d rawYaw = targetVector.getAngle();

    // Get field relative chassis speeds
    Translation2d robotVel =
        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
            .rotateBy(odometryPose.getRotation());
    double fieldVx = robotVel.getX();
    double fieldVy = robotVel.getY();

    // Unit vectors of the shot direction (field frame)
    double ux = rawYaw.getCos();
    double uy = rawYaw.getSin();

    // Components of the robot velocity along the shot-forward and shot-lateral
    // directions
    double parallelComponent = fieldVx * ux + fieldVy * uy; // Velocity along direction of shot
    double perpendicularComponent = fieldVx * -uy + fieldVy * ux; // Velocity perpendicular to shot

    // Flywheel parallel velocity
    double shooterLinear =
        shooterVelocity.in(RadiansPerSecond)
            * ShooterConstants.flywheelRadius.in(Meters)
            * Math.cos(shooterPitch.in(Radians));

    Angle compensationAngle =
        Radians.of(Math.atan2(perpendicularComponent, shooterLinear + parallelComponent));

    return new ShooterControls(
        shooterVelocity, shooterPitch, rawYaw.minus(new Rotation2d(compensationAngle)));
  }
}
