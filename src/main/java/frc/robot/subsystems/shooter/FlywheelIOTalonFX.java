package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

public class FlywheelIOTalonFX implements FlywheelIO {
  protected final TalonFX primaryFlywheelMotor; // Protected for the sim
  protected final TalonFX followerFlywheelMotor;

  private final StatusSignal<AngularVelocity> primaryFlywheelVelocity;
  private final StatusSignal<Voltage> primaryFlywheelAppliedVolts;
  private final StatusSignal<Current> primaryFlywheelCurrent;
  private final StatusSignal<Current> primaryFlywheelTorqueCurrent;

  private final StatusSignal<AngularVelocity> followerFlywheelVelocity;
  private final StatusSignal<Voltage> followerFlywheelAppliedVolts;
  private final StatusSignal<Current> followerFlywheelCurrent;
  private final StatusSignal<Current> followerFlywheelTorqueCurrent;

  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  private final Debouncer primaryConnectedDebounce =
      new Debouncer(ShooterConstants.disconnectedDebounceTimeSec, Debouncer.DebounceType.kFalling);

  private final Debouncer followerConnectedDebounce =
      new Debouncer(ShooterConstants.disconnectedDebounceTimeSec, Debouncer.DebounceType.kFalling);

  /**
   * One TalonFX controlled shooter flywheel
   *
   * @param flywheelCanID This is primarily used to differentiate the two flywheels in logging
   */
  public FlywheelIOTalonFX() {
    primaryFlywheelMotor =
        new TalonFX(ShooterConstants.leftFlywheelMotorID, ShooterConstants.shooterCan);
    followerFlywheelMotor =
        new TalonFX(ShooterConstants.rightFlywheelMotorID, ShooterConstants.shooterCan);

    TalonFXConfiguration primaryFlywheelConfig = new TalonFXConfiguration();

    primaryFlywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.flywheelGearRatio;

    primaryFlywheelConfig.MotorOutput.Inverted = ShooterConstants.leftMotorInverted;
    primaryFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    primaryFlywheelConfig.ClosedLoopGeneral.ContinuousWrap = true;

    primaryFlywheelConfig.Slot0.kP = ShooterConstants.flywheelKP;
    primaryFlywheelConfig.Slot0.kI = 0.0;
    primaryFlywheelConfig.Slot0.kD = ShooterConstants.flywheelKD;

    primaryFlywheelConfig.Slot0.kS = ShooterConstants.flywheelKS;
    primaryFlywheelConfig.Slot0.kV = ShooterConstants.flywheelKV;
    primaryFlywheelConfig.Slot0.kA = ShooterConstants.flywheelKA;

    primaryFlywheelConfig.MotionMagic.MotionMagicAcceleration =
        ShooterConstants.flywheelMaxAcceleration.in(RotationsPerSecondPerSecond);

    primaryFlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    primaryFlywheelConfig.CurrentLimits.SupplyCurrentLimit =
        ShooterConstants.flywheelSupplyCurrentLimitAmps.in(Amps);

    TalonFXConfiguration followerFlywheelConfig = new TalonFXConfiguration();
    followerFlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    followerFlywheelConfig.CurrentLimits.SupplyCurrentLimit =
        ShooterConstants.flywheelSupplyCurrentLimitAmps.in(Amps);

    followerFlywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.flywheelGearRatio;

    // if (Constants.currentMode == Mode.SIM) {
    //   primaryFlywheelConfig.Slot0 = ShooterConstants.simFlywheelConfigs;
    // }
    if (RobotBase.isSimulation()) {
      primaryFlywheelConfig.Slot0 = ShooterConstants.simFlywheelConfigs;
    }
    tryUntilOk(5, () -> primaryFlywheelMotor.getConfigurator().apply(primaryFlywheelConfig, 0.25));
    tryUntilOk(
        5, () -> followerFlywheelMotor.getConfigurator().apply(followerFlywheelConfig, 0.25));

    primaryFlywheelVelocity = primaryFlywheelMotor.getVelocity();
    primaryFlywheelAppliedVolts = primaryFlywheelMotor.getMotorVoltage();
    primaryFlywheelCurrent = primaryFlywheelMotor.getStatorCurrent();
    primaryFlywheelTorqueCurrent = primaryFlywheelMotor.getTorqueCurrent();

    followerFlywheelVelocity = followerFlywheelMotor.getVelocity();
    followerFlywheelAppliedVolts = followerFlywheelMotor.getMotorVoltage();
    followerFlywheelCurrent = followerFlywheelMotor.getStatorCurrent();
    followerFlywheelTorqueCurrent = followerFlywheelMotor.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, primaryFlywheelVelocity, primaryFlywheelAppliedVolts, primaryFlywheelCurrent);

    BaseStatusSignal.setUpdateFrequencyForAll(250.0, primaryFlywheelTorqueCurrent);

    ParentDevice.optimizeBusUtilizationForAll(primaryFlywheelMotor, followerFlywheelMotor);

    followerFlywheelMotor.setControl(
        new Follower(
            ShooterConstants.leftFlywheelMotorID, ShooterConstants.flywheelMotorAlignment));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    primaryFlywheelMotor.setControl(velocityTorqueCurrentRequest.withVelocity(velocity));
  }

  @Override
  public void setOpenLoopTorqueCurrent(Current output) {
    primaryFlywheelMotor.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    StatusCode flywheelStatus =
        BaseStatusSignal.refreshAll(
            primaryFlywheelVelocity,
            primaryFlywheelAppliedVolts,
            primaryFlywheelCurrent,
            primaryFlywheelTorqueCurrent);
    StatusCode followerFlywheelStatus =
        BaseStatusSignal.refreshAll(
            followerFlywheelVelocity,
            followerFlywheelAppliedVolts,
            followerFlywheelCurrent,
            followerFlywheelTorqueCurrent);
    inputs.primaryConnected = primaryConnectedDebounce.calculate(flywheelStatus.isOK());
    inputs.primaryAngularVelocity = primaryFlywheelVelocity.getValue();
    inputs.primaryAppliedVolts = primaryFlywheelAppliedVolts.getValue();
    inputs.primaryStatorCurrentAmps = primaryFlywheelCurrent.getValue();
    inputs.primaryTorqueCurrentAmps = primaryFlywheelTorqueCurrent.getValue();

    inputs.followerConnected = followerConnectedDebounce.calculate(followerFlywheelStatus.isOK());
    inputs.followerAngularVelocity = followerFlywheelVelocity.getValue();
    inputs.followerAppliedVolts = followerFlywheelAppliedVolts.getValue();
    inputs.followerStatorCurrentAmps = followerFlywheelCurrent.getValue();
    inputs.followerTorqueCurrentAmps = followerFlywheelTorqueCurrent.getValue();
  }

  @Override
  public void setPID(Slot0Configs gains) {
    tryUntilOk(5, () -> primaryFlywheelMotor.getConfigurator().apply(gains));
  }
}
