package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Helpers;
import frc.robot.wrappers.RARSparkMax;

public class SwerveModule {
  private final TalonFX m_driveMotor;
  private final RARSparkMax m_turnMotor;
  private final RelativeEncoder m_turningRelEncoder;
  private final SparkAbsoluteEncoder m_turningAbsEncoder;
  private final SimpleMotorFeedforward m_drivingFeedForward;
  private final SparkPIDController m_turningPIDController;

  private final PeriodicIO m_periodicIO = new PeriodicIO();

  private final double m_turningOffset;
  private final String m_moduleName;

  private static class PeriodicIO {
    SwerveModuleState desiredState = new SwerveModuleState();
    boolean shouldChangeState = false;
  }

  private boolean m_moduleDisabled = false;

  public SwerveModule(int driveMotorChannel, int turningMotorChannel, double turningOffset, String moduleName) {
    m_turningOffset = turningOffset;
    m_moduleName = moduleName;

    m_drivingFeedForward = new SimpleMotorFeedforward(
        Constants.SwerveDrive.Drive.k_driveFeedForwardS,
        Constants.SwerveDrive.Drive.k_driveFeedForwardV,
        Constants.SwerveDrive.Drive.k_driveFeedForwardA);

    m_driveMotor = new TalonFX(driveMotorChannel);
    m_driveMotor.setNeutralMode(NeutralModeValue.Coast);
    
    // m_driveEncoder.setVelocityConversionFactor(Units.inchesToMeters(
    //     (Constants.SwerveDrive.k_wheelRadiusIn * 2.0 * Math.PI) / Constants.SwerveDrive.Drive.k_driveGearRatio) / 60.0);
    // m_driveMotor.setSmartCurrentLimit(Constants.Drivetrain.Drive.k_driveCurrentLimit);

    m_turnMotor = new RARSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turnMotor.restoreFactoryDefaults();
    m_turnMotor.setIdleMode(IdleMode.kCoast);
    m_turnMotor.setInverted(true);
    // m_turnMotor.setSmartCurrentLimit(Constants.SwerveDrive.Drive.k_turnCurrentLimit);

    m_turningAbsEncoder = m_turnMotor.getAbsoluteEncoder();

    m_turningRelEncoder = m_turnMotor.getEncoder();
    // m_turningRelEncoder.setPositionConversionFactor(Constants.SwerveDrive.Turn.k_turnGearRatio * 2.0 * Math.PI);
    // m_turningRelEncoder.setVelocityConversionFactor(Constants.SwerveDrive.Turn.k_turnGearRatio * 2.0 * Math.PI / 60.0);

    m_turningPIDController = m_turnMotor.getPIDController();
    m_turningPIDController.setP(Constants.SwerveDrive.Drive.k_driveP);
    m_turningPIDController.setI(Constants.SwerveDrive.Drive.k_driveI);
    m_turningPIDController.setD(Constants.SwerveDrive.Drive.k_driveD);
    // m_turningPIDController.setIZone(Constants.SwerveDrive.Drive.k_driveIZone);
    // m_turningPIDController.setFF(Constants.SwerveDrive.Drive.k_driveFF);

    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(0.0);
    m_turningPIDController.setPositionPIDWrappingMaxInput(2.0 * Math.PI);
    m_turningPIDController.setOutputRange(
        Constants.SwerveDrive.Turn.k_turningMinOutput,
        Constants.SwerveDrive.Turn.k_turningMaxOutput);

    TalonFXConfigurator configurator = m_driveMotor.getConfigurator();
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.Slot0.kP = Constants.SwerveDrive.Drive.k_driveP;
    configuration.Slot0.kI = Constants.SwerveDrive.Drive.k_driveP;
    configuration.Slot0.kD = Constants.SwerveDrive.Drive.k_driveP;
    // configuration.Feedback.RotorToSensorRatio = 
    configuration.Feedback.SensorToMechanismRatio = Units.inchesToMeters(Constants.SwerveDrive.k_wheelRadiusIn * 2.0 * Math.PI)
            / Constants.SwerveDrive.Drive.k_driveGearRatio;
    configurator.apply(configuration);

    // m_driveMotor.burnFlash();
    m_turnMotor.burnFlash();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveVelocity(), Rotation2d.fromRadians(getTurnPosition()));
  }

  public SwerveModulePosition getPosition() {
    double drivePosition = m_driveMotor.getPosition().getValueAsDouble();

    return new SwerveModulePosition(
        drivePosition, Rotation2d.fromRadians(getTurnPosition()));
  }

  public TalonFX getDriveMotor() {
    return m_driveMotor;
  }

  public RARSparkMax getTurnMotor() {
    return m_turnMotor;
  }

  public void clearTurnPIDAccumulation() {
    m_turningPIDController.setIAccum(0);
  }

  public void resetDriveEncoder() {
    m_driveMotor.setPosition(0.0);
  }

  // public void resetTurnConfig() {
  //   if (getAsbEncoderIsConnected()) {
  //     m_turningRelEncoder.setPosition(
  //         Helpers.modRadians(Units.rotationsToRadians(m_turningAbsEncoder.getPosition() - m_turningOffset)));
  //     m_moduleDisabled = false;
  //   }
  // }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(getTurnPosition()));
    desiredState.angle = new Rotation2d(Helpers.modRadians(desiredState.angle.getRadians()));
    m_periodicIO.shouldChangeState = !desiredState.equals(m_periodicIO.desiredState);
    m_periodicIO.desiredState = desiredState;
  }

  // Pass voltage into drive motor and set turn motor to 0 deg
  public void sysidDrive(double volts) {
    m_turningPIDController.setReference(0, ControlType.kPosition);

    m_driveMotor.setVoltage(volts);
  }

  // Pass voltage into turn motor and set drive motor to 0 volts⚡
  public void sysidTurn(double volts) {
    // m_drivePIDController.setReference(0, ControlType.kVoltage);

    m_turnMotor.setVoltage(volts);
  }

  public SwerveModuleState getDesiredState() {
    return m_periodicIO.desiredState;
  }

  public void pointForward() {
    m_periodicIO.desiredState.speedMetersPerSecond = 0.0;
    m_periodicIO.desiredState.angle = new Rotation2d(0.0);
    m_periodicIO.desiredState = SwerveModuleState.optimize(m_periodicIO.desiredState,
        Rotation2d.fromRadians(getTurnPosition()));
    m_periodicIO.shouldChangeState = true;
  }

  public void periodic() {
    if (m_periodicIO.shouldChangeState) {
      if (!m_moduleDisabled) {
        double feedforward = m_drivingFeedForward.calculate(getDriveTargetVelocity());

        // m_drivePIDController.setReference(getDriveTargetVelocity(), ControlType.kVelocity, 0, feedforward);
        m_turningPIDController.setReference(getTurnTargetAngleRadians(), ControlType.kPosition);
      } else {
        DriverStation.reportWarning(m_moduleName + " is disabled, encoder is probably not plugged in!", false);
        m_driveMotor.setNeutralMode(NeutralModeValue.Coast);
        m_turnMotor.setIdleMode(IdleMode.kCoast);
 
        // m_drivePIDController.setReference(0, ControlType.kVoltage);
        m_turningPIDController.setReference(0, ControlType.kVoltage);
      }

      m_periodicIO.shouldChangeState = false;
    }
  }

  // Logged
  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/targetAngle")
  private double getTurnTargetAngleRadians() {
    return m_periodicIO.desiredState.angle.getRadians();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/speedMPS")
  private double getDriveTargetVelocity() {
    return m_periodicIO.desiredState.speedMetersPerSecond;
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/Voltage")
  public double getTurnMotorVoltage() {
    return Helpers.getVoltage(m_turnMotor);
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/Current")
  public double getTurnMotorCurrent() {
    return m_turnMotor.getOutputCurrent();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/Voltage")
  public double getDriveMotorVoltage() {
    return m_driveMotor.getMotorVoltage().getValueAsDouble();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/Current")
  public double getDriveMotorCurrent() {
    return m_driveMotor.getTorqueCurrent().getValueAsDouble();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Abs/getTurnPosition")
  public double getAsbEncoderPosition() {
    return m_turningAbsEncoder.getPosition() - m_turningOffset;
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/Temperature")
  public double getDriveTemp() {
    return m_driveMotor.getDeviceTemp().getValueAsDouble();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/Temperature")
  public double getTurnTemp() {
    return m_turnMotor.getMotorTemperature();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/Velocity")
  public double getDriveVelocity() {
    return m_driveMotor.getVelocity().getValueAsDouble();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/Position")
  public double getDrivePosition() {
    return m_driveMotor.getPosition().getValueAsDouble();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/Position")
  public double getTurnPosition() {
    return Helpers.modRadians(m_turningRelEncoder.getPosition());
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/absPosition")
  public double getTurnAbsPosition() {
    return Helpers.modRotations(m_turningAbsEncoder.getPosition() - m_turningOffset);
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/Velocity")
  public double getTurnVelocity() {
    return m_turningRelEncoder.getVelocity();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/error")
  public double getTurnError() {
    return getState().angle.minus(m_periodicIO.desiredState.angle).getDegrees();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/errorRelToAbs")
  public double getTurnErrorRelToAbs() {
    return getState().angle.minus(Rotation2d.fromRotations(getTurnAbsPosition())).getDegrees();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/ModuleDisabled")
  public boolean isModuleDisabled() {
    return m_moduleDisabled;
  }
}
