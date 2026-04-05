// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  // Logged tunable PID values for drive and turn motors (per-module)
  private final LoggedTunableNumber driveKP;
  private final LoggedTunableNumber driveKI;
  private final LoggedTunableNumber driveKD;
  private final LoggedTunableNumber driveKS;
  private final LoggedTunableNumber driveKV;
  private final LoggedTunableNumber driveKA;

  private final LoggedTunableNumber turnKP;
  private final LoggedTunableNumber turnKI;
  private final LoggedTunableNumber turnKD;
  private final LoggedTunableNumber turnKS;
  private final LoggedTunableNumber turnKV;
  private final LoggedTunableNumber turnKA;

  public Module(
      ModuleIO io,
      int index,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;
    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert(
            "Disconnected turn encoder on module " + Integer.toString(index) + ".",
            AlertType.kError);

    // Initialize tunable numbers using the default gains for this module
    Slot0Configs driveGains = constants.DriveMotorGains;
    driveKP = new LoggedTunableNumber("Drive/Module" + index + "/Drive/kP", driveGains.kP);
    driveKI = new LoggedTunableNumber("Drive/Module" + index + "/Drive/kI", driveGains.kI);
    driveKD = new LoggedTunableNumber("Drive/Module" + index + "/Drive/kD", driveGains.kD);
    driveKS = new LoggedTunableNumber("Drive/Module" + index + "/Drive/kS", driveGains.kS);
    driveKV = new LoggedTunableNumber("Drive/Module" + index + "/Drive/kV", driveGains.kV);
    driveKA = new LoggedTunableNumber("Drive/Module" + index + "/Drive/kA", driveGains.kA);

    Slot0Configs turnGains = constants.SteerMotorGains;
    turnKP = new LoggedTunableNumber("Drive/Module" + index + "/Turn/kP", turnGains.kP);
    turnKI = new LoggedTunableNumber("Drive/Module" + index + "/Turn/kI", turnGains.kI);
    turnKD = new LoggedTunableNumber("Drive/Module" + index + "/Turn/kD", turnGains.kD);
    turnKS = new LoggedTunableNumber("Drive/Module" + index + "/Turn/kS", turnGains.kS);
    turnKV = new LoggedTunableNumber("Drive/Module" + index + "/Turn/kV", turnGains.kV);
    turnKA = new LoggedTunableNumber("Drive/Module" + index + "/Turn/kA", turnGains.kA);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);

    if (Constants.tuningMode) {
      int hash = hashCode();

      // Drive pid
      LoggedTunableNumber.ifChanged(
          hash,
          values -> {
            Slot0Configs gains = new Slot0Configs();
            gains.kP = values[0];
            gains.kI = values[1];
            gains.kD = values[2];
            gains.kS = values[3];
            gains.kV = values[4];
            gains.kA = values[5];
            io.setDrivePID(gains);
          },
          driveKP,
          driveKI,
          driveKD,
          driveKS,
          driveKV,
          driveKA);

      // Turn
      LoggedTunableNumber.ifChanged(
          hash,
          values -> {
            Slot0Configs gains = new Slot0Configs();
            gains.kP = values[0];
            gains.kI = values[1];
            gains.kD = values[2];
            gains.kS = values[3];
            gains.kV = values[4];
            gains.kA = values[5];
            io.setTurnPID(gains);
          },
          turnKP,
          turnKI,
          turnKD,
          turnKS,
          turnKV,
          turnKA);
    }
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPosition);

    // Apply setpoints
    io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius);
    io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(Rotation2d.kZero);

    // Switches between drive and spin motors for sysId
    // io.setTurnOpenLoop(output);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * constants.WheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * constants.WheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }
}
