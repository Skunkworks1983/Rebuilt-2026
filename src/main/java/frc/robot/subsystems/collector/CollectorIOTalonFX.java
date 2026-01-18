package frc.robot.subsystems.collector;

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

public class CollectorIOTalonFX implements CollectorIO {

  private final TalonFX collectorMotor;

  private final Debouncer collectDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  public final StatusSignal<Angle> collectorPosition;
  public final StatusSignal<AngularVelocity> collectorVelocity;
  public final StatusSignal<Voltage> collectorVoltage;
  public final StatusSignal<Current> collectorCurrent;

  public CollectorIOTalonFX(int motorID) {

    collectorMotor = new TalonFX(motorID);

    collectorPosition = collectorMotor.getPosition();
    collectorVelocity = collectorMotor.getVelocity();
    collectorVoltage = collectorMotor.getMotorVoltage();
    collectorCurrent = collectorMotor.getStatorCurrent();
  }

  @Override
  public void updateInputs(CollectorIOInputs inputs) {

    var collectorStatus =
        BaseStatusSignal.refreshAll(
            collectorPosition, collectorVelocity, collectorVoltage, collectorCurrent);

    inputs.connected = collectDebouncer.calculate(collectorStatus.isOK());
    inputs.positionRad = Units.rotationsToRadians(collectorPosition.getValueAsDouble());
    inputs.velocityRad = Units.rotationsToRadians(collectorVelocity.getValueAsDouble());
    inputs.appliedVolts = collectorVoltage.getValueAsDouble();
    inputs.currentAmps = collectorCurrent.getValueAsDouble();
  }

  public void setCollectorVelocity(double speed) {
    collectorMotor.set(speed);
  }
}
