package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelLevel;

public class FlywheelIOTalonFX implements FlywheelIO {
  protected final TalonFX flywheelMotor; // Protected for the sim

  private final StatusSignal<AngularVelocity> flywheelVelocity;
  private final StatusSignal<Voltage> flywheelAppliedVolts;
  private final StatusSignal<Current> flywheelCurrent;
  private final StatusSignal<Current> flywheelTorqueCurrent;

  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  private final Debouncer primaryConnectedDebounce =
      new Debouncer(ShooterConstants.disconnectedDebounceTimeSec, Debouncer.DebounceType.kFalling);

  /**
   * One TalonFX controlled shooter flywheel
   *
   * @param flywheelCanID This is primarily used to differentiate the two flywheels in logging
   */
  public FlywheelIOTalonFX(int flywheelCanID, FlywheelLevel level) {
    flywheelMotor = new TalonFX(flywheelCanID, ShooterConstants.shooterCan);

    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

    flywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.flywheelGearRatio;

    flywheelConfig.MotorOutput.Inverted = ShooterConstants.flywheelMotorInverted;
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    flywheelConfig.ClosedLoopGeneral.ContinuousWrap = true;

    if (level == FlywheelLevel.Upper) {
      flywheelConfig.Slot0.kP = ShooterConstants.upperFlywheelKP;
      flywheelConfig.Slot0.kI = 0.0;
      flywheelConfig.Slot0.kD = ShooterConstants.upperFlywheelKD;

      flywheelConfig.Slot0.kS = ShooterConstants.upperFlywheelKS;
      flywheelConfig.Slot0.kV = ShooterConstants.upperFlywheelKV;
      flywheelConfig.Slot0.kA = ShooterConstants.upperFlywheelKA;
    } else {
      flywheelConfig.Slot0.kP = ShooterConstants.lowerFlywheelKP;
      flywheelConfig.Slot0.kI = 0.0;
      flywheelConfig.Slot0.kD = ShooterConstants.lowerFlywheelKD;

      flywheelConfig.Slot0.kS = ShooterConstants.lowerFlywheelKS;
      flywheelConfig.Slot0.kV = ShooterConstants.lowerFlywheelKV;
      flywheelConfig.Slot0.kA = ShooterConstants.lowerFlywheelKA;
    }

    flywheelConfig.MotionMagic.MotionMagicAcceleration =
        ShooterConstants.flywheelMaxAcceleration.in(RotationsPerSecondPerSecond);

    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConstants.flywheelStatorCurrentLimitAmps.in(Amps);

    if (Constants.currentMode == Mode.SIM) {
      flywheelConfig.Slot0 = ShooterConstants.simFlywheelConfigs;
    }
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
  public void setOpenLoopTorqueCurrent(Current output) {
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
  public void setPID(Slot0Configs gains) {
    tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(gains));
  }
}
