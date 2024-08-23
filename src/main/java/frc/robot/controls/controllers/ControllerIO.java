package frc.robot.controls.controllers;

import org.littletonrobotics.junction.AutoLog;

public interface ControllerIO {
  @AutoLog
  class ControllerIOInputs {
    public boolean[] m_buttonValues = new boolean[] {};
    public double[] m_axisValues = new double[] {};
    public boolean[] m_buttonPressedValues = new boolean[] {};
    public boolean[] m_buttonReleasedValues = new boolean[] {};

    public int m_dPadDir = DPadButton.Direction.UNPRESSED.direction;
  }

  default void updateInputs(ControllerIOInputs inputs) {
  }
}