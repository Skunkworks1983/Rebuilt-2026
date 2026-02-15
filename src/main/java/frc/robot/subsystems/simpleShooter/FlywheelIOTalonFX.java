package frc.robot.subsystems.simpleShooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
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
  private final StatusSignal<Angle> flywheelPosition;

  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);


  private final Debouncer primaryConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public FlywheelIOTalonFX(int flywheelMotorId) {
    flywheelMotor = new TalonFX(flywheelMotorId, ShooterConstants.shooterCan);

    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.flywheelGearRatio;

    flywheelConfig.MotorOutput.Inverted = ShooterConstants.flywheelMotorInverted;

    flywheelConfig.ClosedLoopGeneral.ContinuousWrap = true;

    flywheelConfig.Slot0.kP = ShooterConstants.forwardFlywheelKP;
    flywheelConfig.Slot0.kI = 0.0;
    flywheelConfig.Slot0.kD = ShooterConstants.forwardFlywheelKD;

    flywheelConfig.Slot0.kS = ShooterConstants.forwardFlywheelKS;
    flywheelConfig.Slot0.kV = ShooterConstants.forwardFlywheelKV;
    flywheelConfig.Slot0.kA = ShooterConstants.forwardFlywheelKA;

    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConstants.flywheelStatorCurrentLimitAmps;

    tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(flywheelConfig, 0.25));

    flywheelVelocity = flywheelMotor.getVelocity();
    flywheelAppliedVolts = flywheelMotor.getMotorVoltage();
    flywheelCurrent = flywheelMotor.getStatorCurrent();
    flywheelTorqueCurrent = flywheelMotor.getTorqueCurrent();
    flywheelPosition = flywheelMotor.getPosition();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, flywheelVelocity, flywheelAppliedVolts, flywheelCurrent, flywheelPosition);

    flywheelTorqueCurrent.setUpdateFrequency(
        250); // This is what follower uses to follow, so faser updates
    ParentDevice.optimizeBusUtilizationForAll(flywheelMotor);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    flywheelMotor.setControl(velocityTorqueCurrentRequest.withVelocity(velocity));
  }

  @Override
  public void setOpenLoopCurrent(double output) {
    flywheelMotor.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void setOpenLoopVoltage(double output) {
    flywheelMotor.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    StatusCode flywheelStatus =
        BaseStatusSignal.refreshAll(
            flywheelVelocity, flywheelAppliedVolts, flywheelCurrent, flywheelTorqueCurrent);

    inputs.primaryConnected = primaryConnectedDebounce.calculate(flywheelStatus.isOK());
    inputs.velocity = flywheelVelocity.getValue();

    inputs.primaryAppliedVolts = flywheelAppliedVolts.getValue();

    inputs.primaryStatorCurrentAmps = flywheelCurrent.getValue();

    inputs.primaryTorqueCurrentAmps = flywheelTorqueCurrent.getValue();
    inputs.position = flywheelPosition.getValue();
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;

    tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(config.Slot0));
  }

  @Override
  public void maxDutyCycle() {
    flywheelMotor.setControl(dutyCycleRequest.withOutput(1.0));
  }
}
