package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelIOTalonFX implements FlywheelIO {
  protected final TalonFX flywheelMotor;

  private final StatusSignal<AngularVelocity> flywheelVelocity;
  private final StatusSignal<Voltage> flywheelAppliedVolts;
  private final StatusSignal<Current> flywheelCurrent;
  private final StatusSignal<Current> flywheelTorqueCurrent;

  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final MotionMagicVelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0.0);

  private final Debouncer primaryConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public FlywheelIOTalonFX(int flywheelCanID) {
    flywheelMotor = new TalonFX(flywheelCanID, ShooterConstants.shooterCan);

    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

    flywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.flywheelGearRatio;
    flywheelConfig.MotorOutput.Inverted = ShooterConstants.flywheelMotorInverted;
    flywheelConfig.ClosedLoopGeneral.ContinuousWrap = true;

    flywheelConfig.Slot0.kP = ShooterConstants.flywheelKP;
    flywheelConfig.Slot0.kI = 0.0;
    flywheelConfig.Slot0.kD = ShooterConstants.flywheelKD;

    flywheelConfig.Slot0.kS = ShooterConstants.flywheelKS;
    flywheelConfig.Slot0.kV = ShooterConstants.flywheelKV;
    flywheelConfig.Slot0.kA = ShooterConstants.flywheelKA;

    flywheelConfig.MotionMagic.MotionMagicAcceleration =
        ShooterConstants.flywheelMaxAcceleration.in(RotationsPerSecondPerSecond);

    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConstants.flywheelStatorCurrentLimitAmps.in(Amps);

    tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(flywheelConfig, 0.25));

    flywheelVelocity = flywheelMotor.getVelocity();
    flywheelAppliedVolts = flywheelMotor.getMotorVoltage();
    flywheelCurrent = flywheelMotor.getStatorCurrent();
    flywheelTorqueCurrent = flywheelMotor.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, flywheelVelocity, flywheelAppliedVolts, flywheelCurrent, flywheelTorqueCurrent);

    ParentDevice.optimizeBusUtilizationForAll(flywheelMotor);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    flywheelMotor.setControl(velocityTorqueCurrentRequest.withVelocity(velocity));
  }

  @Override
  public void setOpenLoopOutput(double output) {
    flywheelMotor.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    StatusCode flywheelStatus =
        BaseStatusSignal.refreshAll(
            flywheelVelocity, flywheelAppliedVolts, flywheelCurrent, flywheelTorqueCurrent);

    inputs.connected = primaryConnectedDebounce.calculate(flywheelStatus.isOK());
    inputs.angularVelocity = flywheelVelocity.getValue();
    inputs.appliedVolts = flywheelAppliedVolts.getValue();
    inputs.statorCurrentAmps = flywheelCurrent.getValue();
    inputs.torqueCurrentAmps = flywheelTorqueCurrent.getValue();
  }

  @Override
  public void setMotionConstraints(AngularAcceleration maxAcceleration) {
    MotionMagicConfigs config =
        new TalonFXConfiguration().MotionMagic.withMotionMagicAcceleration(maxAcceleration);

    tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(config));
  }

  @Override
  public void setPID(Slot0Configs gains) {

    tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(gains));
  }
}
