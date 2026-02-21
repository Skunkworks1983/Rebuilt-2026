package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.simpleShooter.ShooterConstants;
import frc.robot.util.FieldConstants.HubCorners;
import org.littletonrobotics.junction.Logger;

public class ShooterCalculations {
    public record ShooterControls(
            AngularVelocity shooterVelocity,
            Angle shooterPitch,
            Rotation2d driveYaw,
            boolean possibleShot) {
    };

    public record NoPitchShooterControls(
            AngularVelocity upperFlywheelVelocity,
            AngularVelocity lowerFlywheelVelocity,
            Rotation2d driveYaw,
            boolean possibleShot) {
    };

    public static ShooterControls getInterpolatedShooterControls(
            ChassisSpeeds speeds, Pose2d odometryPose, ShooterInterporlations shooterInterporlations) {
        Translation3d target = DriverStation.getAlliance().get() == Alliance.Red
                ? FieldConstants.redGoal
                : FieldConstants.blueGoal;

        Translation2d targetVector = target.toTranslation2d().minus(odometryPose.getTranslation());

        Distance distance = Meters.of(targetVector.getNorm());

        AngularVelocity shooterVelocity = shooterInterporlations.getShooterVelocityGoal(distance);
        Angle shooterPitch = shooterInterporlations.getPivotAngleGoal(distance);
        Rotation2d rawYaw = targetVector.getAngle();

        // Get field relative chassis speeds
        Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
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
        double shooterLinear = shooterVelocity.in(RadiansPerSecond)
                * ShooterConstants.flywheelRadius.in(Meters)
                * Math.cos(shooterPitch.in(Radians));

        Angle compensationAngle = Radians.of(Math.atan2(perpendicularComponent, shooterLinear + parallelComponent));

        return new ShooterControls(
                shooterVelocity, shooterPitch, rawYaw.minus(new Rotation2d(compensationAngle)), true);
    }

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
                double distanceToTarget = pose.getTranslation().getDistance(FieldConstants.blueGoal.toTranslation2d());
                for (double initialPitch : prevPitchDegrees) {
                    for (double initialFlywheel : prevFlywheelRotPerSec) {
                        totalCases++;

                        // Get calculated controls
                        ShooterControls out = getNumericalSolvedShooterControls(
                                pose,
                                zeroSpeeds,
                                Degrees.of(initialPitch),
                                RotationsPerSecond.of(initialFlywheel));

                        // Recompute error
                        double deltaZ = FieldConstants.blueGoal.getZ() - ShooterConstants.shooterInitalPosition.getZ();

                        double pitch = out.shooterPitch().in(Radians);
                        double flywheelLinear = out.shooterVelocity().in(RadiansPerSecond)
                                * ShooterConstants.flywheelRadius.in(Meters);

                        double cosTheta = Math.cos(pitch);
                        double sinTheta = Math.sin(pitch);

                        double effectiveVelocity = flywheelLinear * cosTheta;

                        double term1 = (flywheelLinear * sinTheta * distanceToTarget) / effectiveVelocity;
                        double term2 = (ShooterSolverConstants.gravity * distanceToTarget * distanceToTarget)
                                / (2 * effectiveVelocity * effectiveVelocity);

                        double e = term1 - term2 - deltaZ;
                        double absErr = Math.abs(e);
                        if (absErr <= ShooterSolverConstants.maxError)
                            convergedCount++;

                        if (absErr > maxAbsError && out.possibleShot() == true) {
                            maxAbsError = absErr;
                            worstCase = String.format(
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

    public static ShooterControls getApexConstrainedShooterControls(
            Pose2d odometryPose, ChassisSpeeds chassisSpeeds, AngularVelocity gyroRate) {

        Translation3d goal = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red
                        ? FieldConstants.redGoal
                        : FieldConstants.blueGoal;

        Translation2d targetVector = goal.toTranslation2d().minus(odometryPose.getTranslation());

        double trueDistance = targetVector.getNorm();

        double distanceForAngle = Math.max(trueDistance, 1e-6);
        Rotation2d rawYaw = targetVector.times(1.0 / distanceForAngle).getAngle();

        double dragMultiplier = 1.0 + (ShooterSolverConstants.drag * trueDistance);
        dragMultiplier = MathUtil.clamp(dragMultiplier, 1.0, ShooterSolverConstants.maxDragMultiplier);
        double x = trueDistance * dragMultiplier;

        double z = goal.getZ() - ShooterConstants.shooterInitalPosition.getZ();

        double heightTrajectoryPeak = z + ShooterSolverConstants.hubHeightClearance.in(Meters);
        heightTrajectoryPeak = Math.max(heightTrajectoryPeak, 0.1);

        double launchVelocityZ = Math.sqrt(2.0 * ShooterSolverConstants.gravity * heightTrajectoryPeak);

        double timeUp = launchVelocityZ / ShooterSolverConstants.gravity;

        double heightDown = Math.max(heightTrajectoryPeak - z, 0.0);
        double timeDescend = Math.sqrt(2.0 * heightDown / ShooterSolverConstants.gravity);

        double totalTime = timeUp + timeDescend;
        totalTime = Math.max(totalTime, 1e-4);

        double requiredSpeedField = x / totalTime;

        Translation2d requiredFieldVelocity = new Translation2d(
                requiredSpeedField * rawYaw.getCos(), requiredSpeedField * rawYaw.getSin());

        Rotation2d robotRotation = odometryPose.getRotation();

        Translation2d robotLinearVelocityField = new Translation2d(chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond)
                .rotateBy(robotRotation);

        Translation2d rShooterRobot = ShooterConstants.shooterInitalPosition.getTranslation().toTranslation2d();

        double omega = gyroRate.in(RadiansPerSecond);
        Translation2d tangentialRobot = new Translation2d(-omega * rShooterRobot.getY(), omega * rShooterRobot.getX());

        Translation2d tangentialField = tangentialRobot.rotateBy(robotRotation);
        Translation2d totalRobotVelocityField = robotLinearVelocityField.plus(tangentialField);
        Translation2d launchVelocityField = requiredFieldVelocity.minus(totalRobotVelocityField);

        double shooterHorizontalSpeed = Math.max(launchVelocityField.getNorm(), 1e-6);

        double finalPitch = Math.atan2(launchVelocityZ, shooterHorizontalSpeed);

        double ballExitSpeed = Math.hypot(shooterHorizontalSpeed, launchVelocityZ);
        double requiredFlywheelSurfaceSpeed = ballExitSpeed / ShooterSolverConstants.flywheelEfficiency;

        Rotation2d finalYaw = launchVelocityField.getAngle();

        return new ShooterControls(
                RadiansPerSecond.of(
                        requiredFlywheelSurfaceSpeed / ShooterConstants.flywheelRadius.in(Meters)),
                Radians.of(finalPitch),
                finalYaw,
                true);
    }

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

        double velocityRobotAlongShot = chassisSpeeds.vxMetersPerSecond * Math.cos(shotAngleRadians)
                + chassisSpeeds.vyMetersPerSecond * Math.sin(shotAngleRadians);

        double shooterPitch = prevShooterPitch.in(Radians);
        double flywheelLinearVelocity = prevFlywheelVelocity.in(RadiansPerSecond)
                * ShooterConstants.flywheelRadius.in(Meters);

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
            double e = ((flywheelLinearVelocity * sinTheta * x) / effectiveVelocity)
                    - (ShooterSolverConstants.gravity * x * x)
                            / (2 * effectiveVelocity * effectiveVelocity)
                    - z;

            if (Math.abs(e) < ShooterSolverConstants.maxError)
                break;

            double effectiveVelocitySq = effectiveVelocity * effectiveVelocity;
            double effectiveVelocityCu = effectiveVelocitySq * effectiveVelocity;

            // Partial derivative Pitch (theta)
            double slopeTerm_dTheta = (x
                    * flywheelLinearVelocity
                    * (flywheelLinearVelocity + velocityRobotAlongShot * cosTheta))
                    / effectiveVelocitySq;

            double gravityTerm_dTheta = (ShooterSolverConstants.gravity * x * x * (-flywheelLinearVelocity * sinTheta))
                    / effectiveVelocityCu;

            double de_dtheta = slopeTerm_dTheta + gravityTerm_dTheta;

            // Partial derivative Velocity
            double slopeTerm_dV = (x * velocityRobotAlongShot * sinTheta) / effectiveVelocitySq;

            double gravityTerm_dV = (ShooterSolverConstants.gravity * x * x * cosTheta) / effectiveVelocityCu;

            double de_dv = slopeTerm_dV + gravityTerm_dV;

            double dThetaWeighted = de_dtheta * ShooterSolverConstants.weightTheta;
            double dVelocityWeighted = de_dv * ShooterSolverConstants.weightVelocity;

            // Update step using Gradient Descent
            double denominator = (dThetaWeighted * dThetaWeighted + dVelocityWeighted * dVelocityWeighted);

            // Safety epsilon to prevent NaN
            double lambda = e / (denominator + 1e-6);

            shooterPitch -= lambda * dThetaWeighted * ShooterSolverConstants.changeRate;
            flywheelLinearVelocity -= lambda * dVelocityWeighted * ShooterSolverConstants.changeRate;
        }

        // Get field relative chassis speeds
        Translation2d fieldRelativeRobotVel = new Translation2d(chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond)
                .rotateBy(odometryPose.getRotation());

        // Unit vectors of the shot direction (do once to minimize trig)
        double ux = rawYaw.getCos();
        double uy = rawYaw.getSin();

        double parallelComponent = fieldRelativeRobotVel.getX() * ux + fieldRelativeRobotVel.getY() * uy;

        double perpendicularComponent = fieldRelativeRobotVel.getX() * -uy + fieldRelativeRobotVel.getY() * ux;

        Angle compensationAngle = Radians
                .of(Math.atan2(perpendicularComponent, flywheelLinearVelocity + parallelComponent));

        double effectiveVelocity = flywheelLinearVelocity * Math.cos(shooterPitch) + parallelComponent;

        double term1 = (flywheelLinearVelocity * Math.sin(shooterPitch) * x) / effectiveVelocity;
        double term2 = (0.5 * ShooterSolverConstants.gravity * x * x) / (effectiveVelocity * effectiveVelocity);

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

    public static ShooterControls getShooterControlsTransfer(
            ChassisSpeeds speeds, Pose2d odometryPose, ShooterInterporlations shooterInterporlations) {
        Translation2d target = DriverStation.getAlliance().get() == Alliance.Red
                ? FieldConstants.redFuelTransferTarget
                : FieldConstants.blueFuelTransferTarget;
        Translation2d hubCenter = DriverStation.getAlliance().get() == Alliance.Red
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
            Rotation2d hubAngularWidth = new Rotation2d(
                    Radians.of(Math.atan2(FieldConstants.halfHubWidth.in(Meters), robotToHub.getNorm())));

            Rotation2d leftEdgeOption = rawYaw.minus(hubAngularWidth.minus(ShooterSolverConstants.hubSafetyAngle));
            Rotation2d rightEdgeOption = rawYaw.plus(hubAngularWidth.plus(ShooterSolverConstants.hubSafetyAngle));

            // Choose the one closer to the original aim
            rawYaw = Math.abs(rawYaw.minus(leftEdgeOption).getRadians()) < Math
                    .abs(rawYaw.minus(rightEdgeOption).getRadians())
                            ? leftEdgeOption
                            : rightEdgeOption;
        }

        AngularVelocity shooterVelocity = shooterInterporlations.getShooterVelocityTransfer(distance);
        Angle shooterPitch = shooterInterporlations.getPivotAngleTransfer(distance);

        // Get field relative chassis speeds
        Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
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
        double shooterLinear = shooterVelocity.in(RadiansPerSecond)
                * ShooterConstants.flywheelRadius.in(Meters)
                * Math.cos(shooterPitch.in(Radians));

        Angle compensationAngle = Radians.of(Math.atan2(perpendicularComponent, shooterLinear + parallelComponent));

        return new ShooterControls(
                shooterVelocity, shooterPitch, rawYaw.minus(new Rotation2d(compensationAngle)), true);
    }

    public static Rotation2d getShootYaw(
            Pose2d odometryPose,
            ChassisSpeeds chassisSpeeds,
            AngularVelocity gyroRate,
            AngularVelocity shooterSpeed) {
        Translation3d goal = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red
                        ? FieldConstants.redGoal
                        : FieldConstants.blueGoal;

        Translation2d targetVector = goal.toTranslation2d().minus(odometryPose.getTranslation());

        double trueDistance = targetVector.getNorm();

        double distanceForAngle = Math.max(trueDistance, 1e-6);
        Rotation2d rawYaw = targetVector.times(1.0 / distanceForAngle).getAngle();

        double requiredSpeedField = shooterSpeed.in(RadiansPerSecond) * ShooterConstants.flywheelRadius.in(Meters);

        Translation2d requiredFieldVelocity = new Translation2d(
                requiredSpeedField * rawYaw.getCos(), requiredSpeedField * rawYaw.getSin());

        Rotation2d robotRotation = odometryPose.getRotation();

        Translation2d robotLinearVelocityField = new Translation2d(chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond)
                .rotateBy(robotRotation);

        Translation2d rShooterRobot = ShooterConstants.shooterInitalPosition.getTranslation().toTranslation2d();

        double omega = gyroRate.in(RadiansPerSecond);
        Translation2d tangentialRobot = new Translation2d(-omega * rShooterRobot.getY(), omega * rShooterRobot.getX());

        Translation2d tangentialField = tangentialRobot.rotateBy(robotRotation);
        Translation2d totalRobotVelocityField = robotLinearVelocityField.plus(tangentialField);
        Translation2d launchVelocityField = requiredFieldVelocity.minus(totalRobotVelocityField);

        return launchVelocityField.getAngle();
    }

    /**
     * Compute shooter controls for a constrained (fixed) shooter pitch angle.
     *
     * @param odometryPose     current robot pose
     * @param chassisSpeeds    current chassis speeds
     * @param gyroRate         current angular velocity of the robot
     * @param constrainedPitch fixed shooter pitch to use for the shot
     * @return NoPitchShooterControls containing required shooter angular velocity
     *         and drive yaw; the
     *         boolean indicates whether a shot is possible with the provided
     *         constraints
     */
    public static NoPitchShooterControls getNoPitchShooterControls(
            Pose2d odometryPose,
            ChassisSpeeds chassisSpeeds,
            AngularVelocity gyroRate,
            Angle constrainedPitch) {

        Translation3d goal = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red
                        ? FieldConstants.redGoal
                        : FieldConstants.blueGoal;
        Logger.recordOutput("Shooter Solver/Goal", goal);

        Translation2d targetVector = goal.toTranslation2d().minus(odometryPose.getTranslation());

        double trueDistance = targetVector.getNorm();

        Logger.recordOutput("Shooter Solver/Target Distance", trueDistance);

        double distanceForAngle = Math.max(trueDistance, 1e-6);
        Rotation2d rawYaw = targetVector.times(1.0 / distanceForAngle).getAngle();

        double dragMultiplier = 1.0 + (ShooterSolverConstants.drag * trueDistance);
        dragMultiplier = MathUtil.clamp(dragMultiplier, 1.0, ShooterSolverConstants.maxDragMultiplier);
        double x = trueDistance * dragMultiplier;

        double z = goal.getZ() - ShooterConstants.shooterInitalPosition.getZ();

        // Use the provided constrained pitch
        double theta = constrainedPitch.in(Radians);
        double cosTheta = Math.cos(theta);
        double tanTheta = Math.tan(theta);

        // Denominator of the closed form v^2 solution: x*tan(theta) - z
        double denom = x * tanTheta - z;
        if (denom <= 1e-9) {
            // Cannot satisfy projectile equation with this pitch and geometry
            return new NoPitchShooterControls(
                    RotationsPerSecond.of(0), RotationsPerSecond.of(0), rawYaw, false);
        }

        double v2 = (ShooterSolverConstants.gravity * x * x) / (2.0 * cosTheta * cosTheta * denom);
        if (Double.isNaN(v2) || v2 <= 0.0) {
            return new NoPitchShooterControls(
                    RotationsPerSecond.of(0), RotationsPerSecond.of(0), rawYaw, false);
        }

        double ballExitSpeed = Math.sqrt(v2);

        double requiredFieldHorizontal = ballExitSpeed * cosTheta;

        Translation2d requiredFieldVelocity = new Translation2d(
                requiredFieldHorizontal * rawYaw.getCos(), requiredFieldHorizontal * rawYaw.getSin());

        Rotation2d robotRotation = odometryPose.getRotation();

        Translation2d robotLinearVelocityField = new Translation2d(chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond)
                .rotateBy(robotRotation);

        Translation2d rShooterRobot = ShooterConstants.shooterInitalPosition.getTranslation().toTranslation2d();

        double omega = gyroRate.in(RadiansPerSecond);
        Translation2d tangentialRobot = new Translation2d(-omega * rShooterRobot.getY(), omega * rShooterRobot.getX());

        Translation2d tangentialField = tangentialRobot.rotateBy(robotRotation);
        Translation2d totalRobotVelocityField = robotLinearVelocityField.plus(tangentialField);
        Translation2d launchVelocityField = requiredFieldVelocity.minus(totalRobotVelocityField);

        Rotation2d finalYaw = launchVelocityField.getAngle();

        double requiredFlywheelSurfaceSpeed = ballExitSpeed / ShooterSolverConstants.flywheelEfficiency;

        return new NoPitchShooterControls(
                RadiansPerSecond.of(
                        requiredFlywheelSurfaceSpeed / ShooterConstants.flywheelRadius.in(Meters)),
                RadiansPerSecond.of(
                        requiredFlywheelSurfaceSpeed / ShooterConstants.flywheelRadius.in(Meters)),
                finalYaw,
                true);
    }

    /** Compute shooter controls for a constrained (fixed) shooter pitch angle. */
    public static NoPitchShooterControls getNoPitchExtrapolatedControls(
            Pose2d odometryPose,
            ChassisSpeeds chassisSpeeds, // kept for signature compatibility; no longer used
            Angle constrainedPitch) {

        Translation3d goal = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red
                        ? FieldConstants.redGoal
                        : FieldConstants.blueGoal;
        Logger.recordOutput("Shooter Solver/Goal", goal);

        // 1) Extrapolate robot pose to release time (same pattern as Apex code)
        Pose2d releasePose = odometryPose.exp(
                new Twist2d(
                        chassisSpeeds.vxMetersPerSecond * ShooterSolverConstants.defaultShotLatencySec,
                        chassisSpeeds.vyMetersPerSecond * ShooterSolverConstants.defaultShotLatencySec,
                        chassisSpeeds.omegaRadiansPerSecond
                                * ShooterSolverConstants.defaultShotLatencySec));

        Translation2d shooterOffsetField = ShooterConstants.shooterInitalPosition
                .getTranslation()
                .toTranslation2d()
                .rotateBy(releasePose.getRotation());

        Translation2d releaseShooterPosField = releasePose.getTranslation().plus(shooterOffsetField);

        // 2) Target vector from release shooter position
        Translation2d targetVector = goal.toTranslation2d().minus(releaseShooterPosField);
        double trueDistance = targetVector.getNorm();
        Logger.recordOutput("Shooter Solver/Target Distance", trueDistance);

        Rotation2d rawYaw = targetVector.getAngle();

        // Drag approximation
        Translation2d chassisVelField = new Translation2d(chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond)
                .rotateBy(releasePose.getRotation());

        double speedTowardsTarget = (chassisVelField.getX() * rawYaw.getCos())
                + (chassisVelField.getY() * rawYaw.getSin());

        double adjustedDistance = trueDistance
                - (speedTowardsTarget * ShooterSolverConstants.extrapolatedControllerEfficency.get());
        double dragMultiplier = 1.0 + (ShooterSolverConstants.drag * trueDistance);

        dragMultiplier = MathUtil.clamp(dragMultiplier, 1.0, ShooterSolverConstants.maxDragMultiplier);
        double x = adjustedDistance * dragMultiplier;

        double z = goal.getZ() - ShooterConstants.shooterInitalPosition.getZ();

        // Use the provided constrained pitch
        double theta = constrainedPitch.in(Radians);
        double cosTheta = Math.cos(theta);
        double tanTheta = Math.tan(theta);

        // Denominator of the closed form v^2 solution: x*tan(theta) - z
        double denom = x * tanTheta - z;
        if (denom <= 1e-9) {
            return new NoPitchShooterControls(
                    RotationsPerSecond.of(0), RotationsPerSecond.of(0), rawYaw, false);
        }

        double v2 = (ShooterSolverConstants.gravity * x * x) / (2.0 * cosTheta * cosTheta * denom);
        if (Double.isNaN(v2) || v2 <= 0.0) {
            return new NoPitchShooterControls(
                    RotationsPerSecond.of(0), RotationsPerSecond.of(0), rawYaw, false);
        }

        double ballExitSpeed = Math.sqrt(v2);

        // Horizontal component required if robot were stationary
        double requiredFieldHorizontal = ballExitSpeed * cosTheta;

        Translation2d stationaryLaunchVectorField = new Translation2d(
                requiredFieldHorizontal * rawYaw.getCos(), requiredFieldHorizontal * rawYaw.getSin());

        double omega = chassisSpeeds.omegaRadiansPerSecond;

        Translation2d tangentialVelField = new Translation2d(-omega * shooterOffsetField.getY(),
                omega * shooterOffsetField.getX());

        Translation2d shooterVelField = chassisVelField.plus(tangentialVelField);

        // 4) Compensate
        Translation2d effectiveLaunchVectorField = stationaryLaunchVectorField.minus(shooterVelField);
        Rotation2d finalYaw = effectiveLaunchVectorField.getAngle();

        Rotation2d yawError = finalYaw.minus(rawYaw);
        finalYaw = rawYaw.plus(yawError.times(ShooterSolverConstants.yawCompensationMultiplier.get()));

        double requiredFlywheelSurfaceSpeed = ballExitSpeed
                / ShooterSolverConstants.extrapolatedControllerEfficency.get();

        double estFlightTime = x / Math.max(requiredFieldHorizontal, 1e-4);
        Translation2d leadShiftField = chassisVelField.times(estFlightTime);

        Translation3d virtualGoal = new Translation3d(
                goal.getX() - leadShiftField.getX(), goal.getY() - leadShiftField.getY(), goal.getZ());

        Translation2d virtualTargetVector = virtualGoal.toTranslation2d().minus(releaseShooterPosField);

        Logger.recordOutput("Shooter Solver/Goal/Real", goal);
        Logger.recordOutput("Shooter Solver/Goal/Virtual", virtualGoal);
        Logger.recordOutput("Shooter Solver/Goal/LeadShiftXY", leadShiftField);

        Logger.recordOutput("Shooter Solver/ReleasePose", releasePose);
        Logger.recordOutput("Shooter Solver/ReleaseShooterPosField", releaseShooterPosField);

        Logger.recordOutput("Shooter Solver/TargetVector/Real", targetVector);
        Logger.recordOutput("Shooter Solver/TargetVector/Virtual", virtualTargetVector);

        Logger.recordOutput("Shooter Solver/StationaryLaunchVectorField", stationaryLaunchVectorField);
        Logger.recordOutput("Shooter Solver/ChassisVelField", chassisVelField);
        Logger.recordOutput("Shooter Solver/TangentialVelField", tangentialVelField);
        Logger.recordOutput("Shooter Solver/ShooterVelField", shooterVelField);
        Logger.recordOutput("Shooter Solver/EffectiveLaunchVectorField", effectiveLaunchVectorField);

        Logger.recordOutput("Shooter Solver/Yaw/Raw", rawYaw);
        Logger.recordOutput("Shooter Solver/Yaw/FinalUnscaled", effectiveLaunchVectorField.getAngle());
        Logger.recordOutput("Shooter Solver/Yaw/CompErrorRad", yawError.getRadians());
        Logger.recordOutput("Shooter Solver/Yaw/FinalScaled", finalYaw);

        Translation2d reconstructedStationary = effectiveLaunchVectorField.plus(shooterVelField);
        Logger.recordOutput(
                "Shooter Solver/StationaryMinusReconstructedError",
                reconstructedStationary.minus(stationaryLaunchVectorField).getNorm());

        Logger.recordOutput("Shooter Solver/DragMultiplier", dragMultiplier);
        Logger.recordOutput("Shooter Solver/DistWithDrag", x);
        Logger.recordOutput("Shooter Solver/BallExitSpeed", ballExitSpeed);
        Logger.recordOutput("Shooter Solver/EstFlightTime", estFlightTime);

        return new NoPitchShooterControls(
                RadiansPerSecond.of(
                        requiredFlywheelSurfaceSpeed / ShooterConstants.flywheelRadius.in(Meters)),
                RadiansPerSecond.of(
                        (requiredFlywheelSurfaceSpeed
                                * ShooterSolverConstants.lowerFlywheelVelocityMultiplier.get())
                                / ShooterConstants.flywheelRadius.in(Meters)),
                finalYaw,
                true);
    }

    /**
     * Calculates shooter controls for the "transfer" shot (shooting into the
     * alliance zone from the
     * transfer zone). Includes logic to avoid hitting the hub structure.
     *
     * <p>
     * It is attempted to first calculate shooter pitch from the current velocity
     * (beause we can
     * change pitch faster than velocity and it uses less amperage) If this fails,
     * we set pitch to be
     * the midpoint of the possible pitch range, and caluclate needed flywheel
     * velocity (this ensures
     * that we minimize having to change flywheel velocity)
     *
     * <h2>Implementation details</h2>
     *
     * <ol>
     * <li>Selects the transfer target and the hub (blocking geometry) corner points
     * based on
     * alliance.
     * <li>Computes the raw drive yaw and straight-line distance from the robot pose
     * to the target.
     * <li>Computes the robot's angle interval blocked by the hub by finding the
     * most CW and most
     * CCW corner angles, and also track the nearest hub-corner distance.
     * <li>Decides if the hub blocks the shot line
     * <li>If blocked, pick the smaller-magnitude yaw adjustment to clear the hub
     * (with a safety
     * margin)
     * <li>If yaw is adjusted, recompute an effective distance by intersecting the
     * new yaw ray with
     * the target’s X coordinate (only if the intersection is in front of the
     * robot).
     * <li>Ballistic solve (range equation): first try to compute pitch using the
     * current flywheel
     * linear speed; choose low-arc if valid, otherwise high-arc if valid.
     * <li>If no valid pitch exists at the current speed, solve for the required
     * flywheel speed
     * using the midpoint pitch
     * </ol>
     *
     * @param speeds                 Current robot chassis speeds.
     * @param odometryPose           Current robot pose.
     * @param flywheelLinearVelocity Current flywheel linear velocity
     * @return Calculated {@link ShooterControls}.
     */
    public static NoPitchShooterControls getShooterControlsTransfer(
            ChassisSpeeds speeds, Pose2d odometryPose, LinearVelocity flywheelLinearVelocity) {
        Translation2d target = DriverStation.getAlliance().get() == Alliance.Red
                ? FieldConstants.redFuelTransferTarget
                : FieldConstants.blueFuelTransferTarget;

        HubCorners hubCorners = DriverStation.getAlliance().get() == Alliance.Red
                ? FieldConstants.redHubCorners
                : FieldConstants.blueHubCorners;
        Translation2d targetVector = target.minus(odometryPose.getTranslation());
        Logger.recordOutput("TransferShot/TargetVector", targetVector);
        Logger.recordOutput("TransferShot/Target", target);
        Rotation2d rawYaw = targetVector.getAngle();
        Distance distanceToTarget = Meters.of(targetVector.getNorm());

        Rotation2d finalYaw = rawYaw;

        // PART 1: YAW/ACTUAL TARGET CALCULATER (Hub avoidance)
        Translation2d[] corners = new Translation2d[] {
                hubCorners.getAllianceSideLeft(),
                hubCorners.getAllianceSideRight(),
                hubCorners.getNeutralSideLeft(),
                hubCorners.getNeutralSideRight()
        };

        Angle hubFurthestCCWAngle = Radians.of(-Double.MAX_VALUE); // Most positive (CCW) angle to hub
        Angle hubFurthestCWAngle = Radians.of(Double.MAX_VALUE); // Most negative (CW) angle to hub
        Distance minDistToHub = Meters.of(Double.MAX_VALUE);

        for (Translation2d corner : corners) {
            Translation2d robotToCorner = corner.minus(odometryPose.getTranslation());

            Distance distToCorner = Meters.of(robotToCorner.getNorm());
            if (distToCorner.lt(minDistToHub)) {
                minDistToHub = distToCorner;
            }

            Rotation2d angleToCorner = robotToCorner.getAngle().minus(rawYaw); // Using Rotation2d to handle angle
            // wrapping

            Angle angleToCornerMeasure = Radians.of(angleToCorner.getRadians());

            if (angleToCornerMeasure.gt(hubFurthestCCWAngle))
                hubFurthestCCWAngle = angleToCornerMeasure;
            if (angleToCornerMeasure.lt(hubFurthestCWAngle))
                hubFurthestCWAngle = angleToCornerMeasure;
        }

        // 5. Collision Detection & Avoidance Logic
        boolean hubIsCloser = minDistToHub.lt(distanceToTarget);
        boolean lineOfSightBlocked = hubFurthestCWAngle.lt(Radians.of(0)) && hubFurthestCCWAngle.gt(Radians.of(0));

        if (hubIsCloser && lineOfSightBlocked) {
            Angle rightCandidate = hubFurthestCWAngle.minus(ShooterSolverConstants.transferShotSafetyMargin);
            Angle leftCandidate = hubFurthestCCWAngle.plus(ShooterSolverConstants.transferShotSafetyMargin);

            // Pick the smaller magnitude adjustment
            if (Math.abs(rightCandidate.in(Radians)) <= Math.abs(leftCandidate.in(Radians))) {
                finalYaw = rawYaw.plus(Rotation2d.fromRadians(rightCandidate.in(Radians)));
            } else {
                finalYaw = rawYaw.plus(Rotation2d.fromRadians(leftCandidate.in(Radians)));
            }

            double cosYaw = finalYaw.getCos();
            double netTargetDx = target.getX() - odometryPose.getTranslation().getX();

            if (Math.abs(cosYaw) > 1e-3) {
                double distanceToNewTarget = netTargetDx / cosYaw;

                if (distanceToNewTarget > 0.0) { // intersection is in front of robot
                    distanceToTarget = Meters.of(distanceToNewTarget);
                }
            }
        }

        // PART 2: BALLISTIC SOLVER
        // This solves the standard projectile range equation
        double distanceToTargetMeters = distanceToTarget.in(Meters);
        double currentFlywheelLienarVelocity = flywheelLinearVelocity.in(MetersPerSecond);

        double calculatedFlywheelVelocity = currentFlywheelLienarVelocity;
        double sinTwoTheta = Math.sin(2.0 * ShooterConstants.shooterPitch.in(Radians));

        double requiredV = Math.sqrt((ShooterSolverConstants.gravity * distanceToTargetMeters) / sinTwoTheta);
        calculatedFlywheelVelocity = requiredV;

        return new NoPitchShooterControls(
                RadiansPerSecond.of(
                        calculatedFlywheelVelocity / ShooterConstants.flywheelRadius.in(Meters)),
                RadiansPerSecond.of(
                        calculatedFlywheelVelocity / ShooterConstants.flywheelRadius.in(Meters)),
                finalYaw,
                true);
    }
}
