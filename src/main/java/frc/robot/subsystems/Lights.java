// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;

/** Add your docs here. */
public class Lights {

  DigitalOutput led0;
  DigitalOutput led1;
  DigitalOutput led2;

  LightPattern currentLight = LightPattern.None;

  public Lights() {
    led0 = new DigitalOutput(0);
    led1 = new DigitalOutput(0);
    led2 = new DigitalOutput(0);
  }

  public void setLight(LightPattern pattern) {
    currentLight = pattern;
    led0.set((pattern.getValue() & 1) > 0);
    led1.set((pattern.getValue() & 2) > 0);
    led2.set((pattern.getValue() & 4) > 0);
  }

  public LightPattern getPattern() {
    return currentLight;
  }

  public enum LightPattern {
    None(0),
    Pattern1(1),
    Pattern2(2),
    Pattern3(3),
    Pattern4(4),
    Pattern5(5),
    Pattern6(6),
    Pattern7(7);

    private final int value;

    LightPattern(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }
  }
}
