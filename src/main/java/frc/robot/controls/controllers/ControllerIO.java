package frc.robot.controls.controllers;

import org.littletonrobotics.junction.AutoLog;

public interface ControllerIO {
  @AutoLog
  class ControllerIOInputs {
    public boolean[] m_buttonValues = new boolean[] { false, false, false, false, false, false, false, false,
        false, false };
    public double[] m_axisValues = new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    public boolean[] m_buttonPressedValues = new boolean[] { false, false, false, false, false, false, false, false,
        false, false };
    public boolean[] m_buttonReleasedValues = new boolean[] { false, false, false, false, false, false, false, false,
        false, false };

    public int m_dPadDir = DPadButton.Direction.UNPRESSED.direction;
  }

  default void updateInputs(ControllerIOInputs inputs) {
  }
}