package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class PivotIOTalonFX implements PivotIO {
  protected final TalonFX pivotMotor;
  protected final CANcoder pivotEncoder;

  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> pivotCurrent;
  private final StatusSignal<Current> pivotTorqueCurrent;
  protected final StatusSignal<Double> closedLoopError;
  private final StatusSignal<Angle> pivotAbsolutePosition;
  private final StatusSignal<AngularVelocity> pivotVelocity;

  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final MotionMagicTorqueCurrentFOC positionTorqueCurentRequest =
      new MotionMagicTorqueCurrentFOC(0);

  private final Debouncer motorConnectedDebounce =
      new Debouncer(ShooterConstants.disconnectedDebounceTimeSec, Debouncer.DebounceType.kFalling);

  private final Debouncer encoderConnectedDebounce =
      new Debouncer(ShooterConstants.disconnectedDebounceTimeSec, Debouncer.DebounceType.kFalling);

  public PivotIOTalonFX() {
    pivotMotor = new TalonFX(ShooterConstants.pivotMotorId, ShooterConstants.shooterCan);
    pivotEncoder = new CANcoder(ShooterConstants.pivotEncoderId, ShooterConstants.shooterCan);
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = ShooterConstants.pivotEncoderTotalOffset;
    encoderConfig.MagnetSensor.SensorDirection = ShooterConstants.pivotEncoderDirection;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    tryUntilOk(5, () -> pivotEncoder.getConfigurator().apply(encoderConfig, 0.25));

    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

    pivotConfig.MotorOutput.Inverted = ShooterConstants.pivotMotorInverted;
    pivotConfig.ClosedLoopGeneral.ContinuousWrap = true;

    pivotConfig.Slot0.kP = ShooterConstants.pivotkP;
    pivotConfig.Slot0.kI = 0.0;
    pivotConfig.Slot0.kD = ShooterConstants.pivotkD;

    pivotConfig.Slot0.kS = ShooterConstants.pivotkS;
    pivotConfig.Slot0.kV = ShooterConstants.pivotkV;
    pivotConfig.Slot0.kA = ShooterConstants.pivotkA;

    pivotConfig.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConstants.pivotMaxVelocity.in(RotationsPerSecond);
    pivotConfig.MotionMagic.MotionMagicAcceleration =
        ShooterConstants.pivotMaxAcceleration.in(RotationsPerSecondPerSecond);

    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConstants.pivotStatorCurrentLimit.in(Amps);

    pivotConfig.Feedback.SensorToMechanismRatio = ShooterConstants.pivotSensorToMechanismRatio;
    pivotConfig.Feedback.RotorToSensorRatio = ShooterConstants.pivotMotorToSensorRatio;
    // pivotConfig.Feedback.SensorToMechanismRatio = 400;
    pivotConfig.Feedback.FeedbackRemoteSensorID = ShooterConstants.pivotEncoderId;
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    // pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    // pivotConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    // pivotConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue =
    //     ShooterConstants.pivotStartAngle.in(Rotations);

    pivotConfig.HardwareLimitSwitch.ForwardLimitSource = ShooterConstants.pivotForwardLimitSource;
    pivotConfig.HardwareLimitSwitch.ReverseLimitSource = ShooterConstants.pivotReverseLimitSource;
    pivotConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
    pivotConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    pivotConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = ShooterConstants.shooterCandiId;
    pivotConfig.HardwareLimitSwitch.ForwardLimitRemoteSensorID = ShooterConstants.shooterCandiId;

    tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotConfig, 0.25));

    // pivotMotor.setPosition(ShooterConstants.pivotStartAngle);
    pivotAbsolutePosition = pivotEncoder.getAbsolutePosition();
    pivotPosition = pivotMotor.getPosition();
    pivotVelocity = pivotMotor.getVelocity();
    pivotAppliedVolts = pivotMotor.getMotorVoltage();
    pivotCurrent = pivotMotor.getStatorCurrent();
    pivotTorqueCurrent = pivotMotor.getTorqueCurrent();
    closedLoopError = pivotMotor.getClosedLoopError();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotPosition,
        pivotAbsolutePosition,
        pivotVelocity,
        pivotAppliedVolts,
        pivotCurrent,
        pivotTorqueCurrent,
        closedLoopError);
    ParentDevice.optimizeBusUtilizationForAll(
        pivotMotor); // For reason known to God and God alone if you put the
    // specific encoder mounted on the robot in this method it
    // breaks
  }

  @Override
  public void setPosition(Angle position) {
    pivotMotor.setControl(positionTorqueCurentRequest.withPosition(position));
  }

  @Override
  public void setOpenLoopTorqueCurrent(Current current) {
    pivotMotor.setControl(torqueCurrentRequest.withOutput(current));
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    StatusCode pivotMotorStatus =
        BaseStatusSignal.refreshAll(
            pivotPosition, pivotAppliedVolts, pivotCurrent, pivotTorqueCurrent, pivotVelocity);
    StatusCode pivotEncoderStatus =
        BaseStatusSignal.refreshAll(pivotAbsolutePosition, pivotVelocity);

    inputs.motorConnected = motorConnectedDebounce.calculate(pivotMotorStatus.isOK());
    inputs.encoderConnected = encoderConnectedDebounce.calculate(pivotEncoderStatus.isOK());
    inputs.pivotAbsolutePosition = pivotAbsolutePosition.getValue();
    inputs.pivotPosition = pivotPosition.getValue();
    inputs.pivotVelocity = pivotVelocity.getValue();
    inputs.pivotAppliedVolts = pivotAppliedVolts.getValue();
    inputs.pivotStatorCurrentAmps = pivotCurrent.getValue();
    inputs.pivotTorqueCurrentAmps = pivotTorqueCurrent.getValue();
  }

  @Override
  public void setMotionConstraints(
      AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
    MotionMagicConfigs config =
        new TalonFXConfiguration()
            .MotionMagic.withMotionMagicCruiseVelocity(maxVelocity)
                .withMotionMagicAcceleration(maxAcceleration);
    tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(config));
  }

  @Override
  public void setPID(Slot0Configs gains) {
    tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(gains));
  }
}
