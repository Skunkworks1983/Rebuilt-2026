// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class BeambreakIOSensor implements BeamBreakIO {

  DigitalInput beamBreakSensor;

  public final boolean beamBroken;

  Debouncer beambreakDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public BeambreakIOSensor(int channel) {
    beamBreakSensor = new DigitalInput(channel);
    beamBroken = beamBreakSensor.get();
  }

  @Override
  public void updateInputs(BeamBreakIOInputs inputs) {
    inputs.beamBroken = beamBroken;
    inputs.connected = beambreakDebouncer.calculate(beamBroken);
  }
}
