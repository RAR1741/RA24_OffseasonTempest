package frc.robot.controls.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.controls.Deadband;
import frc.robot.controls.SquaredInput;

public class FilteredController implements ControllerIO {
  private static final double DEADBAND_LIMIT = 0.03;

  private boolean m_useDeadband;
  private boolean m_useSquaredInput;

  public double k_allianceMultiplier = -1.0;

  private ControllerIOInputsAutoLogged inputs = new ControllerIOInputsAutoLogged();
  private GenericHID m_hid;
  private Deadband m_deadband = new Deadband(DEADBAND_LIMIT);
  private SquaredInput m_squaredInput = new SquaredInput(DEADBAND_LIMIT);

  private final DPadButton[] hatButtons = { new DPadButton(this, DPadButton.Direction.UP),
      new DPadButton(this, DPadButton.Direction.DOWN), new DPadButton(this, DPadButton.Direction.LEFT),
      new DPadButton(this, DPadButton.Direction.RIGHT) };

  public FilteredController(int port) {
    m_hid = new GenericHID(port);
    m_useDeadband = false;
    m_useSquaredInput = false;
  }

  public FilteredController(int port, boolean useDeadband, boolean useSquaredInput) {
    this(port);
    this.m_useDeadband = useDeadband;
    this.m_useSquaredInput = useSquaredInput;
  }

  public void setAllianceMultiplier() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      k_allianceMultiplier = -1.0;
    } else {
      k_allianceMultiplier = 1.0;
    }
  }

  public double getFilteredAxis(int axis) {
    double value = inputs.m_axisValues[axis];

    // Apply squared input, if requested
    if (m_useSquaredInput) {
      value = m_squaredInput.scale(value);
    }

    // Apply deadband, if requested
    if (m_useDeadband) {
      value = m_deadband.scale(value);
    }

    return value;
  }

  public boolean getButton(int button) {
    return inputs.m_buttonValues[button];
  }

  public boolean getButtonPressed(int button) {
    return inputs.m_buttonReleasedValues[button];
  }

  public boolean getButtonReleased(int button) {
    return inputs.m_buttonReleasedValues[button];
  }

  public int getPOV() {
    return inputs.m_dPadDir;
  }

  public boolean getHatPressed(int direction) {
    return hatButtons[direction].getPressed();
  }

  public boolean getHat(int direction) {
    return hatButtons[direction].get();
  }

  public void periodic() {
    this.updateInputs(inputs);
  }

  public interface Button {
    int A = 1;
    int B = 2;
    int X = 3;
    int Y = 4;
    int LEFT_BUMPER = 5;
    int RIGHT_BUMPER = 6;
    int BACK = 7;
    int START = 8;
    int LEFT_JOYSTICK = 9;
    int RIGHT_JOYSTICK = 10;
  }

  public interface Axis {
    int LEFT_X_AXIS = 0;
    int LEFT_Y_AXIS = 1;
    int LEFT_TRIGGER = 2;
    int RIGHT_TRIGGER = 3;
    int RIGHT_X_AXIS = 4;
    int RIGHT_Y_AXIS = 5;
  }

  public interface DPadDirection {
    int UP = 0;
    int DOWN = 1;
    int LEFT = 2;
    int RIGHT = 3;
  }

  @Override
  public void updateInputs(ControllerIOInputs inputs) {
    for (int i = m_hid.getAxisCount(); i > 0; i--) {
      inputs.m_axisValues[i] = m_hid.getRawAxis(i);
    }

    for (int i = m_hid.getButtonCount(); i > 0; i--) {
      inputs.m_buttonValues[i] = m_hid.getRawButton(i);
      inputs.m_buttonPressedValues[i] = m_hid.getRawButtonPressed(i);
      inputs.m_buttonReleasedValues[i] = m_hid.getRawButtonReleased(i);
    }

    inputs.m_dPadDir = m_hid.getPOV();
  }
}
