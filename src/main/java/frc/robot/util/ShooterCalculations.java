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
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

/**
 * Utility class for calculating shooter setpoints based on field geometry and robot state. Includes
 * both analytical and iterative solver methods.
 *
 * @author Eddy Walsh
 */
public class ShooterCalculations {
  /**
   * Data record representing the calculated control outputs for the shooter subsystem.
   *
   * @param shooterVelocity Target flywheel angular velocity.
   * @param shooterPitch Target pivot angle.
   * @param driveYaw Target robot chassis heading.
   * @param possibleShot Whether the shot is physically possible given the constraints.
   */
  public record ShooterControls(
      AngularVelocity shooterVelocity,
      Angle shooterPitch,
      Rotation2d driveYaw,
      boolean possibleShot) {}

  /**
   * Calculates shooter controls for aiming at the goal using linear interpolation. Compensates for
   * robot velocity to enable shooting while moving.
   *
   * @param speeds Current robot chassis speeds
   * @param odometryPose Current robot pose on the field.
   * @param shooterInterporlations Lookup table helper for goal shots.
   * @return Calculated {@link ShooterControls}.
   */
  public static ShooterControls getShooterControlsGoal(
      ChassisSpeeds speeds, Pose2d odometryPose, ShooterInterpolation shooterInterporlations) {
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

  /**
   * Runs a convergence test for the numerical solver across the field to verify stability. Prints
   * results to stdout.
   *
   * @param stepMeters Grid step size for the test in meters.
   * @param prevPitchDegrees Array of initial pitch guesses to test.
   * @param prevFlywheelRotPerSec Array of initial flywheel velocity guesses to test.
   */
  public static void runOptimizedSolverConvergenceTest(
      double stepMeters, double[] prevPitchDegrees, double[] prevFlywheelRotPerSec) {

    int totalCases = 0;
    int convergedCount = 0;
    double maxAbsError = 1;
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
            if (absErr <= ShooterSolverConstants.maxError) convergedCount++;

            if (absErr > maxAbsError && out.possibleShot() == true) {
              maxAbsError = absErr;
              worstCase =
                  String.format(
                      "pose=(%.2f,%.2f) initPitch=%.3f initFlywheel=%.2f -> finalPitch=%.3f finalFlywheelLin=%.3f absErr=%.6f",
                      x, y, initialPitch, initialFlywheel, pitch, flywheelLinear, absErr);
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

  /**
   * Calculates shooter controls using an apex-constrained ballistic model. Assumes the shot peaks
   * at a specific height clearance above the goal.
   *
   * @param odometryPose Current robot pose.
   * @param chassisSpeeds Current robot chassis speeds.
   * @param gyroRate Current robot gyro rotation rate.
   * @return Calculated {@link ShooterControls}.
   */
  public static ShooterControls getApexConstrainedShooterControls(
      Pose2d odometryPose, ChassisSpeeds robotRelativeSpeeds) {
    
    Rotation2d currentRobotRotation = odometryPose.getRotation();

    Translation2d robotLinearVelocityField =
        new Translation2d(
                robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond)
            .rotateBy(currentRobotRotation);

    Translation2d futureTranslation =
        odometryPose.getTranslation().plus(robotLinearVelocityField.times(ShooterSolverConstants.defaultShotLatencySec));

    Rotation2d futureRotation =
        currentRobotRotation.plus(
            Rotation2d.fromRadians(robotRelativeSpeeds.omegaRadiansPerSecond * ShooterSolverConstants.defaultShotLatencySec));

    Pose2d futurePose = new Pose2d(futureTranslation, futureRotation);

    Translation3d goal =
        DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
            ? FieldConstants.redGoal
            : FieldConstants.blueGoal;

    Translation2d shooterOffsetField =
        ShooterConstants.shooterInitalPosition
            .getTranslation()
            .toTranslation2d()
            .rotateBy(futurePose.getRotation());

    Translation2d futureShooterPosField = futurePose.getTranslation().plus(shooterOffsetField);

    Translation2d targetVector = goal.toTranslation2d().minus(futureShooterPosField);
    double trueDistance = targetVector.getNorm();

    // Simple drag approximation
    double dragMultiplier = 1.0 + (ShooterSolverConstants.drag * trueDistance);
    dragMultiplier = MathUtil.clamp(dragMultiplier, 1.0, ShooterSolverConstants.maxDragMultiplier);
    double distWithDrag = trueDistance * dragMultiplier;

    double relativeGoalZ = goal.getZ() - ShooterConstants.shooterInitalPosition.getZ();

    // Apex height calculation
    double heightTrajectoryPeak = relativeGoalZ + ShooterSolverConstants.hubHeightClearance.in(Meters);
    heightTrajectoryPeak = Math.max(heightTrajectoryPeak, 0.1);

    double launchVelocityZ = Math.sqrt(2.0 * ShooterSolverConstants.gravity * heightTrajectoryPeak);

    double timeUp = launchVelocityZ / ShooterSolverConstants.gravity;
    double heightDown = Math.max(heightTrajectoryPeak - relativeGoalZ, 0.0);
    double timeDescend = Math.sqrt(2.0 * heightDown / ShooterSolverConstants.gravity);

    double totalTime = timeUp + timeDescend;
    totalTime = Math.max(totalTime, 1e-4);

    // Speed required if the robot were standing still
    double requiredHorizontalSpeed = distWithDrag / totalTime;

    // Angle
    Rotation2d angleToGoal = targetVector.getAngle();
    Translation2d stationaryLaunchVectorField =
        new Translation2d(requiredHorizontalSpeed, angleToGoal);

    // Chassis speeds compensation
    Translation2d effectiveLaunchVectorField =
        stationaryLaunchVectorField.minus(robotLinearVelocityField);

    Rotation2d finalDriveYaw = effectiveLaunchVectorField.getAngle();

    double effectiveHorizontalSpeed = effectiveLaunchVectorField.getNorm();
    double ballExitSpeed = Math.hypot(effectiveHorizontalSpeed, launchVelocityZ);
    double flywheelSurfaceSpeed = ballExitSpeed / ShooterSolverConstants.flywheelEfficiency;

    double finalPitch = Math.atan2(launchVelocityZ, effectiveHorizontalSpeed);

    return new ShooterControls(
        RadiansPerSecond.of(flywheelSurfaceSpeed / ShooterConstants.flywheelRadius.in(Meters)),
        Radians.of(finalPitch),
        finalDriveYaw,
        true);
  }

  /**
   * Calculates shooter controls using an iterative numerical solver (Gradient Descent). Solves for
   * pitch and velocity simultaneously to minimize error.
   *
   * @param odometryPose Current robot pose.
   * @param chassisSpeeds Current robot chassis speeds.
   * @param prevShooterPitch Previous pitch setpoint for seeding the solver.
   * @param prevFlywheelVelocity Previous flywheel velocity setpoint for seeding the solver.
   * @return Calculated {@link ShooterControls}.
   */
  public static ShooterControls getNumericalSolvedShooterControls(
      Pose2d odometryPose,
      ChassisSpeeds chassisSpeeds,
      Angle prevShooterPitch,
      AngularVelocity prevFlywheelVelocity) {
    // Translation3d target = DriverStation.getAlliance().get() == Alliance.Red
    // ? FieldConstants.redGoal
    // : FieldConstants.blueGoal;

    Translation3d target = FieldConstants.blueGoal;

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

    double effectiveVelocity = flywheelLinearVelocity * Math.cos(shooterPitch) + parallelComponent;

    double term1 = (flywheelLinearVelocity * Math.sin(shooterPitch) * x) / effectiveVelocity;
    double term2 =
        (0.5 * ShooterSolverConstants.gravity * x * x) / (effectiveVelocity * effectiveVelocity);

    double e = term1 - term2 - z;

    if (e > ShooterSolverConstants.maxError) {
      return new ShooterControls(
          prevFlywheelVelocity,
          prevShooterPitch,
          rawYaw.minus(new Rotation2d(compensationAngle)),
          true);
    }

    return new ShooterControls(
        RadiansPerSecond.of(flywheelLinearVelocity / ShooterConstants.flywheelRadius.in(Meters)),
        Radians.of(shooterPitch),
        rawYaw.minus(new Rotation2d(compensationAngle)),
        true);
  }

  /**
   * Calculates shooter controls for the "transfer" shot (shooting into the alliance zone from the
   * transfer zone). Includes logic to avoid hitting the hub structure.
   *
   * @param speeds Current robot chassis speeds.
   * @param odometryPose Current robot pose.
   * @param shooterInterporlations Lookup table helper for transfer shots.
   * @return Calculated {@link ShooterControls}.
   */
  public static ShooterControls getShooterControlsTransfer(
      ChassisSpeeds speeds, Pose2d odometryPose, ShooterInterpolation shooterInterporlations) {
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
}
