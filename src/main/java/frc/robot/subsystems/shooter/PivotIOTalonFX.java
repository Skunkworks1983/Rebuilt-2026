package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static frc.robot.util.PhoenixUtil.tryUntilOk;
import frc.robot.subsystems.shooter.FlywheelIO.FlywheelIOInputs;

public class PivotIOTalonFX implements PivotIO {

  private final TalonFX pivotMotor;
  private final TalonFX pivotFollower;

  private final StatusSignal<Angle> pivotPostion;
  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> pivotCurrent;
  private final StatusSignal<Current> pivotTorqueCurrent;

  private final StatusSignal<Angle> pivotFollowerPosition;
  private final StatusSignal<Voltage> pivotFollowerAppliedVolts;
  private final StatusSignal<Current> pivotFollowerCurrent;
  private final StatusSignal<Current> pivotFollowerTorqueCurrent;

  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final MotionMagicTorqueCurrentFOC postionTorqueCurrentRequest = new MotionMagicTorqueCurrentFOC(
      0.0);

  private final Debouncer primaryConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer followerConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public PivotIOTalonFX() {
    pivotMotor = new TalonFX(ShooterConstants.flywheelMotorOneId, ShooterConstants.shooterCan);
    pivotFollower =
        new TalonFX(ShooterConstants.flywheelSecondaryMotorId, ShooterConstants.shooterCan);

    pivotFollower.setControl(
        new Follower(pivotMotor.getDeviceID(), ShooterConstants.flywheelFollowerMotorAlignment));

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
        ShooterConstants.flywheelMaxAccelerationRotationsPerSec;
    flywheelConfig.MotionMagic.MotionMagicJerk =
        ShooterConstants.flywheelMaxJerkRotationsPerSecCubed;

    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConstants.flywheelStatorCurrentLimitAmps;

    tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(flywheelConfig, 0.25));
    tryUntilOk(5, () -> pivotFollower.getConfigurator().apply(new TalonFXConfiguration(), 0.25));

    pivotPostion = pivotMotor.getPosition();
    pivotAppliedVolts = pivotMotor.getMotorVoltage();
    pivotCurrent = pivotMotor.getStatorCurrent();
    pivotTorqueCurrent = pivotMotor.getTorqueCurrent();

    pivotFollowerPosition = pivotFollower.getPosition();
    pivotFollowerAppliedVolts = pivotFollower.getMotorVoltage();
    pivotFollowerCurrent = pivotFollower.getStatorCurrent();
    pivotFollowerTorqueCurrent = pivotFollower.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotPostion,
        pivotAppliedVolts,
        pivotCurrent,
        pivotFollowerPosition,
        pivotFollowerAppliedVolts,
        pivotFollowerCurrent,
        pivotFollowerTorqueCurrent);

    pivotTorqueCurrent.setUpdateFrequency(
        250); // This is what follower uses to follow, so faser updates
    ParentDevice.optimizeBusUtilizationForAll(pivotMotor, pivotFollower);
  }

  @Override
  public void setPosition(Angle position) {
    pivotMotor.setControl(postionTorqueCurrentRequest.withPosition(position));
  }

  @Override
  public void setOpenLoopCurrent(Current current) {
    pivotMotor.setControl(torqueCurrentRequest.withOutput(current));
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    StatusCode flywheelStatus = BaseStatusSignal.refreshAll(
        pivotPostion, pivotAppliedVolts, pivotCurrent, pivotTorqueCurrent);
    StatusCode flywheelFollowerStatus = BaseStatusSignal.refreshAll(
        pivotFollowerPosition,
        pivotFollowerAppliedVolts,
        pivotFollowerCurrent,
        pivotFollowerTorqueCurrent);

    inputs.primaryConnected = primaryConnectedDebounce.calculate(flywheelStatus.isOK());
    inputs.followerConnected = followerConnectedDebounce.calculate(flywheelFollowerStatus.isOK());

    inputs.pivotPrimaryPosition = pivotPostion.getValue();
    inputs.pivotFollowerPosition = pivotFollowerPosition.getValue();

    inputs.pivotAppliedVolts = pivotAppliedVolts.getValue();
    inputs.pivotFollowerAppliedVolts = pivotFollowerAppliedVolts.getValue();

    inputs.pivotStatorCurrentAmps = pivotCurrent.getValue();
    inputs.pivotFollowerStatorCurrentAmps = pivotFollowerCurrent.getValue();

    inputs.pivotTorqueCurrentAmps = pivotTorqueCurrent.getValue();
    inputs.pivotFollowerTorqueCurrentAmps = pivotFollowerTorqueCurrent.getValue();
  }

  @Override
  public void setMotionConstraints(AngularVelocity maxVelocity, AngularAcceleration maxAcceleration, double maxJerk) {
    MotionMagicConfigs config = new TalonFXConfiguration().MotionMagic
    .withMotionMagicCruiseVelocity(maxVelocity)
    .withMotionMagicAcceleration(maxAcceleration)
    .withMotionMagicJerk(maxJerk);


    tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(config));
    tryUntilOk(5, () -> pivotFollower.getConfigurator().apply(config));
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

    tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(config.Slot0));
    tryUntilOk(5, () -> pivotFollower.getConfigurator().apply(config.Slot0));
  }
}
