package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class PivotIOTalonFX implements PivotIO {
  protected final TalonFX pivotMotor;

  private final StatusSignal<Angle> pivotPostion;
  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> pivotCurrent;
  private final StatusSignal<Current> pivotTorqueCurrent;
  protected final StatusSignal<Double> closedLoopError;

  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurentRequest =
      new PositionTorqueCurrentFOC(0);

  private final Debouncer primaryConnectedDebounce =
      new Debouncer(ShooterConstants.disconnectedDebounceTimeSec, Debouncer.DebounceType.kFalling);

  public PivotIOTalonFX() {
    pivotMotor = new TalonFX(ShooterConstants.pivotMotorId, ShooterConstants.shooterCan);

    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

    pivotConfig.MotorOutput.Inverted = ShooterConstants.pivotMotorInverted;
    pivotConfig.ClosedLoopGeneral.ContinuousWrap = true;

    pivotConfig.Slot0.kP = ShooterConstants.pivotkP;
    pivotConfig.Slot0.kI = 0.0;
    pivotConfig.Slot0.kD = ShooterConstants.pivotkD;

    pivotConfig.Slot0.kS = ShooterConstants.pivotkS;
    pivotConfig.Slot0.kV = ShooterConstants.pivotkV;
    pivotConfig.Slot0.kA = ShooterConstants.pivotkA;

    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConstants.pivotStatorCurrentLimit.in(Amps);

    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ShooterConstants.minShooterPitch.in(Rotations);
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ShooterConstants.maxShooterPitch.in(Rotations);
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    pivotConfig.Feedback.SensorToMechanismRatio = ShooterConstants.pivotSensorToHoodRotations;

    tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotConfig, 0.25));
    tryUntilOk(
        5,
        () ->
            pivotMotor.setPosition(
                angleToHoodAngle(ShooterConstants.pivotInitialAngle).in(Rotations)));
    pivotPostion = pivotMotor.getPosition();
    pivotAppliedVolts = pivotMotor.getMotorVoltage();
    pivotCurrent = pivotMotor.getStatorCurrent();
    pivotTorqueCurrent = pivotMotor.getTorqueCurrent();
    closedLoopError = pivotMotor.getClosedLoopError();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, pivotPostion, pivotAppliedVolts, pivotCurrent, pivotTorqueCurrent, closedLoopError);

    ParentDevice.optimizeBusUtilizationForAll(pivotMotor);
  }

  @Override
  public void setAngle(Angle position) {
    pivotMotor.setControl(positionTorqueCurentRequest.withPosition(angleToHoodAngle(position)));
  }

  @Override
  public void setOpenLoopTorqueCurrent(Current current) {
    pivotMotor.setControl(torqueCurrentRequest.withOutput(current));
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    StatusCode pivotStatus =
        BaseStatusSignal.refreshAll(
            pivotPostion, pivotAppliedVolts, pivotCurrent, pivotTorqueCurrent);

    inputs.connected = primaryConnectedDebounce.calculate(pivotStatus.isOK());
    inputs.pivotRawPosition = pivotPostion.getValue();
    inputs.pivotAppliedVolts = pivotAppliedVolts.getValue();
    inputs.pivotStatorCurrentAmps = pivotCurrent.getValue();
    inputs.pivotTorqueCurrentAmps = pivotTorqueCurrent.getValue();
    inputs.pivotConvertedPosition = talonAngleToHoodAngle(pivotPostion.getValue());
  }

  @Override
  public void setPID(Slot0Configs gains) {
    tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(gains));
  }

  public Angle angleToHoodAngle(Angle fieldAngle) {
    return Degrees.of(
        ShooterConstants.maxShooterPitch.in(Degrees)
            + ShooterConstants.minShooterPitch.in(Degrees)
            - fieldAngle.in(Degrees));
  }

  public Angle talonAngleToHoodAngle(Angle talonAngle) {
    return Degrees.of(
        (ShooterConstants.maxShooterPitch.in(Degrees)
                + ShooterConstants.minShooterPitch.in(Degrees))
            - talonAngle.in(Degrees));
  }
}
