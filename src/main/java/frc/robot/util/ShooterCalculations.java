package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.shooter.ShooterConstants;

public class ShooterCalculations {
  public record ShooterControls(
      AngularVelocity shooterVelocity,
      Angle shooterPitch,
      Rotation2d driveYaw,
      boolean possibleShot) {}
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
        shooterVelocity, shooterPitch, rawYaw.minus(new Rotation2d(compensationAngle)), true);
  }

  public static void runOptimizedSolverConvergenceTest(
      double stepMeters, double[] prevPitchDegrees, double[] prevFlywheelRotPerSec) {

    int totalCases = 0;
    int convergedCount = 0;
    double maxAbsError = 0;
    String worstCase = "";

    double minX = 0;
    double minY = 0;

    double maxX = Units.inchesToMeters(156.11);
    double maxY = Units.inchesToMeters(317.69);

    ChassisSpeeds zeroSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    for (double x = minX; x <= maxX; x += stepMeters) {
      for (double y = minY; y <= maxY; y += stepMeters) {
        Pose2d pose = new Pose2d(x, y, new Rotation2d());
        double distanceToTarget =
            pose.getTranslation().getDistance(FieldConstants.blueGoal.toTranslation2d());
        for (double initialPitch : prevPitchDegrees) {
          for (double initialFlywheel : prevFlywheelRotPerSec) {
            totalCases++;

            // Get calculated controls
            ShooterControls out =
                getNumericalSolvedShooterControls(
                    pose,
                    zeroSpeeds,
                    Degrees.of(initialPitch),
                    RotationsPerSecond.of(initialFlywheel));

            // Recompute error
            double deltaZ =
                FieldConstants.blueGoal.getZ() - ShooterConstants.shooterInitalPosition.getZ();

            double pitch = out.shooterPitch().in(Radians);
            double flywheelLinear =
                out.shooterVelocity().in(RadiansPerSecond)
                    * ShooterConstants.flywheelRadius.in(Meters);

            double cosTheta = Math.cos(pitch);
            double sinTheta = Math.sin(pitch);

            double effectiveVelocity = flywheelLinear * cosTheta;

            double term1 = (flywheelLinear * sinTheta * distanceToTarget) / effectiveVelocity;
            double term2 =
                (ShooterSolverConstants.gravity * distanceToTarget * distanceToTarget)
                    / (2 * effectiveVelocity * effectiveVelocity);

            double e = term1 - term2 - deltaZ;
            double absErr = Math.abs(e);
            if (absErr <= ShooterSolverConstants.maxError) {
              convergedCount++;
            }

            if (absErr > maxAbsError) {
              maxAbsError = absErr;
              worstCase =
                  String.format(
                      "pose=(%.2f,%.2f) initPitch=%.3f initFlywheel=%.2f -> finalPitch=%.3f deg finalFlywheelLin=%.3f m/s absErr=%.6f",
                      x,
                      y,
                      initialPitch,
                      initialFlywheel,
                      Math.toDegrees(pitch),
                      flywheelLinear,
                      absErr);
            }
          }
        }
      }
    }

    System.out.println("=== Optimized Solver Convergence Test Summary ===");
    System.out.println("Total cases: " + totalCases);
    System.out.println("Converged (|residual| <= maxError): " + convergedCount);
    System.out.println(
        "Convergence rate: " + (100.0 * convergedCount / Math.max(1, totalCases)) + "%");
    System.out.println("Max absolute residual encountered: " + maxAbsError + " meters");
    System.out.println("Worst case: " + worstCase);
    System.out.println("Max error threshold used: " + ShooterSolverConstants.maxError);
  }

  public static ShooterControls getApexConstrainedShooterControls(
      Pose2d odometryPose, ChassisSpeeds chassisSpeeds) {

    // 1. Setup
    Translation3d goal =
        DriverStation.getAlliance().get() == Alliance.Red
            ? FieldConstants.redGoal
            : FieldConstants.blueGoal;

    Translation2d targetVector = goal.toTranslation2d().minus(odometryPose.getTranslation());

    double trueDistance = targetVector.getNorm();

    double x = trueDistance * (1.0 + (ShooterSolverConstants.drag * trueDistance));

    double z =
        goal.getZ()
            - ShooterConstants.shooterInitalPosition.getZ(); // Height diff between goal and shooter

    Rotation2d rawYaw = targetVector.getAngle();

    double heightTrajectoryPeak = z + ShooterSolverConstants.hubHeightClearance.in(Meters);
    heightTrajectoryPeak = Math.max(heightTrajectoryPeak, 0.1);

    // Vertical velocity needed to reach apex hPeak
    double launchVelocityZ = Math.sqrt(2 * ShooterSolverConstants.gravity * heightTrajectoryPeak);

    // Time to ascend from shooter z to hub height z
    double timeUp = launchVelocityZ / ShooterSolverConstants.gravity;

    // Time to descend from peak to goal height
    double timeDescend = Math.sqrt(2 * (heightTrajectoryPeak - z) / ShooterSolverConstants.gravity);

    double totalTime = timeUp + timeDescend;

    // Velocity in the direction of the goal
    double requiredSpeedField = x / totalTime;

    Translation2d requiredFieldVelocity = new Translation2d(requiredSpeedField, 0).rotateBy(rawYaw);

    // Tangential velocity = omega * r (perpendicular to robot heading)
    Translation2d tangentialVelocity =
        new Translation2d(
                0,
                chassisSpeeds.omegaRadiansPerSecond
                    * ShooterConstants.shooterInitalPosition
                        .getTranslation()
                        .toTranslation2d()
                        .getNorm())
            .rotateBy(odometryPose.getRotation());

    Translation2d robotLinearVelocity =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .rotateBy(odometryPose.getRotation());

    Translation2d robotFieldVelocity =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .rotateBy(odometryPose.getRotation());

    Translation2d totalRobotVelocity = robotLinearVelocity.plus(tangentialVelocity);

    Translation2d launchVelocityRobot = requiredFieldVelocity.minus(totalRobotVelocity);

    double shooterHorizontalSpeed = launchVelocityRobot.getNorm();

    double finalPitch = Math.atan2(launchVelocityZ, shooterHorizontalSpeed);
    double finalVelocity = Math.hypot(shooterHorizontalSpeed, launchVelocityZ);

    Rotation2d finalYaw = launchVelocityRobot.getAngle();

    return new ShooterControls(
        RadiansPerSecond.of((finalVelocity / ShooterConstants.flywheelRadius.in(Meters))),
        Radians.of(finalPitch),
        finalYaw, // Removed the .plus(rawYaw)
        true);
  }

  public static ShooterControls getNumericalSolvedShooterControls(
      Pose2d odometryPose,
      ChassisSpeeds chassisSpeeds,
      Angle prevShooterPitch,
      AngularVelocity prevFlywheelVelocity) {
    Translation3d target =
        DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
            ? FieldConstants.redGoal
            : FieldConstants.blueGoal;

    Translation2d targetVector = target.toTranslation2d().minus(odometryPose.getTranslation());

    // Intentionally move everything to doubles for math in iterative steps
    double x = targetVector.getNorm();
    double z = target.getZ() - ShooterConstants.shooterInitalPosition.getZ();

    Rotation2d rawYaw = targetVector.getAngle();
    double shotAngleRadians = rawYaw.getRadians();

    // Get field relative chassis speeds
    Translation2d fieldRelativeRobotVel =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .rotateBy(odometryPose.getRotation());

    double velocityRobotAlongShot =
        fieldRelativeRobotVel.getX() * Math.cos(shotAngleRadians)
            + fieldRelativeRobotVel.getY() * Math.sin(shotAngleRadians);

    double shooterPitch = prevShooterPitch.in(Radians);
    double flywheelLinearVelocity =
        prevFlywheelVelocity.in(RadiansPerSecond) * ShooterConstants.flywheelRadius.in(Meters);

    if (x < ShooterSolverConstants.minShootDistanceHub.in(Meters)) {
      return new ShooterControls(prevFlywheelVelocity, prevShooterPitch, rawYaw, false);
    }

    for (int i = 0; i < ShooterSolverConstants.maxIterations; i++) {
      // Trig once for optimization
      double cosTheta = Math.cos(shooterPitch);
      double sinTheta = Math.sin(shooterPitch);

      // Velocity of flywheel + robot velocity in flywheel direction
      double effectiveVelocity = flywheelLinearVelocity * cosTheta + velocityRobotAlongShot;

      // Error of current pitch and velocity from target
      double e =
          ((flywheelLinearVelocity * sinTheta * x) / effectiveVelocity)
              - (ShooterSolverConstants.gravity * x * x)
                  / (2 * effectiveVelocity * effectiveVelocity)
              - z;

      if (Math.abs(e) < ShooterSolverConstants.maxError) break;

      double effectiveVelocitySq = effectiveVelocity * effectiveVelocity;
      double effectiveVelocityCu = effectiveVelocitySq * effectiveVelocity;

      // Partial derivative Pitch (theta)
      double slopeTerm_dTheta =
          (x
                  * flywheelLinearVelocity
                  * (flywheelLinearVelocity + velocityRobotAlongShot * cosTheta))
              / effectiveVelocitySq;

      double gravityTerm_dTheta =
          (ShooterSolverConstants.gravity * x * x * (-flywheelLinearVelocity * sinTheta))
              / effectiveVelocityCu;

      double de_dtheta = slopeTerm_dTheta + gravityTerm_dTheta;

      // Partial derivative Velocity
      double slopeTerm_dV = (x * velocityRobotAlongShot * sinTheta) / effectiveVelocitySq;

      double gravityTerm_dV =
          (ShooterSolverConstants.gravity * x * x * cosTheta) / effectiveVelocityCu;

      double de_dv = slopeTerm_dV + gravityTerm_dV;

      double dThetaWeighted = de_dtheta * ShooterSolverConstants.weightTheta;
      double dVelocityWeighted = de_dv * ShooterSolverConstants.weightVelocity;

      // Update step using Gradient Descent
      double denominator =
          (dThetaWeighted * dThetaWeighted + dVelocityWeighted * dVelocityWeighted);

      // Safety epsilon to prevent NaN
      double lambda = e / (denominator + 1e-6);

      shooterPitch -= lambda * dThetaWeighted * ShooterSolverConstants.changeRate;
      flywheelLinearVelocity -= lambda * dVelocityWeighted * ShooterSolverConstants.changeRate;

      // shooterPitch = MathUtil.clamp(
      // shooterPitch,
      // ShooterConstants.minShooterPitch.in(Radians),
      // ShooterConstants.maxShooterPitch.in(Radians));
      // flywheelLinearVelocity = MathUtil.clamp(flywheelLinearVelocity, 0,
      // ShooterConstants.flywheelMaxLinearVelocity.in(MetersPerSecond));

    }

    double horizontalShotSpeed = flywheelLinearVelocity * Math.cos(shooterPitch);

    double ux = rawYaw.getCos();
    double uy = rawYaw.getSin();

    double perpendicularComponent =
        fieldRelativeRobotVel.getX() * -uy + fieldRelativeRobotVel.getY() * ux;

    double ratio = -perpendicularComponent / horizontalShotSpeed;

    ratio = MathUtil.clamp(ratio, -1.0, 1.0);

    Angle compensationAngle = Radians.of(Math.asin(ratio));

    double parallelComponent =
        fieldRelativeRobotVel.getX() * ux + fieldRelativeRobotVel.getY() * uy;

    double effectiveVelocity =
        horizontalShotSpeed * Math.cos(compensationAngle.in(Radians)) + parallelComponent;

    double term1 = (flywheelLinearVelocity * Math.sin(shooterPitch) * x) / effectiveVelocity;
    double term2 =
        (0.5 * ShooterSolverConstants.gravity * x * x) / (effectiveVelocity * effectiveVelocity);

    double e = term1 - term2 - z;

    if (e > ShooterSolverConstants.maxError) {
      return new ShooterControls(
          prevFlywheelVelocity,
          prevShooterPitch,
          rawYaw.minus(new Rotation2d(compensationAngle)),
          false);
    }

    return new ShooterControls(
        RadiansPerSecond.of(flywheelLinearVelocity / ShooterConstants.flywheelRadius.in(Meters)),
        Radians.of(shooterPitch),
        rawYaw.plus(new Rotation2d(compensationAngle)),
        true);
  }

  public static ShooterControls getShooterControlsTransfer(
      ChassisSpeeds speeds, Pose2d odometryPose, ShooterInterporlations shooterInterporlations) {
    Translation2d target =
        DriverStation.getAlliance().get() == Alliance.Red
            ? FieldConstants.redFuelTransferTarget
            : FieldConstants.blueFuelTransferTarget;
    Translation2d hubCenter =
        DriverStation.getAlliance().get() == Alliance.Red
            ? FieldConstants.redHubCenter
            : FieldConstants.blueHubCenter;

    Translation2d targetVector = target.minus(odometryPose.getTranslation());

    Rotation2d rawYaw = targetVector.getAngle();
    Distance distance = Meters.of(targetVector.getNorm());

    Translation2d robotToHub = hubCenter.minus(odometryPose.getTranslation());
    Translation2d hubInShotFrame = robotToHub.rotateBy(rawYaw.unaryMinus());

    double xHub = hubInShotFrame.getX();
    double yHub = hubInShotFrame.getY();

    boolean hitsHub = xHub > 0 && Math.abs(yHub) < FieldConstants.halfHubWidth.in(Meters);

    if (hitsHub) {
      Rotation2d hubAngularWidth =
          new Rotation2d(
              Radians.of(Math.atan2(FieldConstants.halfHubWidth.in(Meters), robotToHub.getNorm())));

      Rotation2d leftEdgeOption =
          rawYaw.minus(hubAngularWidth.minus(ShooterSolverConstants.hubSafetyAngle));
      Rotation2d rightEdgeOption =
          rawYaw.plus(hubAngularWidth.plus(ShooterSolverConstants.hubSafetyAngle));

      // Choose the one closer to the original aim
      rawYaw =
          Math.abs(rawYaw.minus(leftEdgeOption).getRadians())
                  < Math.abs(rawYaw.minus(rightEdgeOption).getRadians())
              ? leftEdgeOption
              : rightEdgeOption;
    }

    AngularVelocity shooterVelocity = shooterInterporlations.getShooterVelocityTransfer(distance);
    Angle shooterPitch = shooterInterporlations.getPivotAngleTransfer(distance);

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
        shooterVelocity, shooterPitch, rawYaw.minus(new Rotation2d(compensationAngle)), true);
  }

  public static ShooterControls getShooterControlsLookupHybrid(
      Pose2d odometryPose, ChassisSpeeds chassisSpeeds) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    Translation3d goal =
        (alliance == Alliance.Red) ? FieldConstants.redGoal : FieldConstants.blueGoal;

    Translation2d robotToGoal = goal.toTranslation2d().minus(odometryPose.getTranslation());
    double dist = robotToGoal.getNorm();

    double timeOfFlight = 0.5;
    Translation2d virtualGoal = new Translation2d();

    for (int i = 0; i < 4; i++) {
      virtualGoal =
          robotToGoal.minus(
              new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
                  .times(timeOfFlight));

      dist = virtualGoal.getNorm();

      double targetSpeed = ShooterInterporlations.shooterGoalVelocityMap.get(dist);
      timeOfFlight = dist / (targetSpeed * 0.8); // 0.8 accounts for arc path
    }

    double targetFlywheelSpeed = ShooterInterporlations.shooterGoalVelocityMap.get(dist);

    double x = virtualGoal.getNorm(); // Horizontal distance
    double z = goal.getZ() - ShooterConstants.shooterInitalPosition.getZ(); // Vertical distance
    double v = targetFlywheelSpeed;
    double g = 9.81;

    double v2 = v * v;
    double termA = (g * x * x) / (2 * v2);
    double termB = -x;
    double termC = z + termA;

    double discriminant = (termB * termB) - (4 * termA * termC);

    if (discriminant < 0) {
      // Unreachable at this velocity!
      return new ShooterControls(
          RadiansPerSecond.of(targetFlywheelSpeed),
          ShooterSolverConstants.defaultPitch,
          virtualGoal.getAngle(),
          false);
    }

    // Choose the highs angle for better entry
    double tanTheta = (-termB + Math.sqrt(discriminant)) / (2 * termA);
    double pitchAngle = Math.atan(tanTheta);

    return new ShooterControls(
        RadiansPerSecond.of(targetFlywheelSpeed / ShooterConstants.flywheelRadius.in(Meters)),
        Radians.of(pitchAngle),
        virtualGoal.getAngle(), // Yaw points at virtual target
        true);
  }
}
