// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class IndexerIOTalonFX implements IndexerIO {

  private final TalonFX indexerMotor;

  private final Debouncer indexerDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  public final StatusSignal<Angle> indexerPosition;
  public final StatusSignal<AngularVelocity> indexerVelocity;
  public final StatusSignal<Voltage> indexerVoltage;
  public final StatusSignal<Current> indexerCurrent;

  public IndexerIOTalonFX(int motorID) {

    indexerMotor = new TalonFX(motorID);

    indexerPosition = indexerMotor.getPosition();
    indexerVelocity = indexerMotor.getVelocity();
    indexerVoltage = indexerMotor.getMotorVoltage();
    indexerCurrent = indexerMotor.getStatorCurrent();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {

    var IndexerStatus =
        BaseStatusSignal.refreshAll(
            indexerPosition, indexerVelocity, indexerVoltage, indexerCurrent);

    inputs.connected = indexerDebouncer.calculate(IndexerStatus.isOK());
    inputs.positionRad = Units.rotationsToRadians(indexerPosition.getValueAsDouble());
    inputs.velocityRad = Units.rotationsToRadians(indexerVelocity.getValueAsDouble());
    inputs.appliedVolts = indexerVoltage.getValueAsDouble();
    inputs.currentAmps = indexerCurrent.getValueAsDouble();
  }

  public void setIndexerSpeed(double speed) { //replace the .set with something better later
    indexerMotor.set(speed);
  }
}
