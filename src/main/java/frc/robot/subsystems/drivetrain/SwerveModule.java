package frc.robot.subsystems.drivetrain;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularAcceleration;
import frc.robot.Constants;
import frc.robot.Helpers;
import frc.robot.wrappers.RARSparkMax;

public class SwerveModule {
  private final TalonFX m_driveMotor;
  private final RARSparkMax m_turnMotor;
  
  private final SparkClosedLoopController m_turningPIDController;
  private final SparkAbsoluteEncoder m_turningAbsEncoder;
  private final RelativeEncoder m_turningRelEncoder;

  private final PeriodicIO m_periodicIO = new PeriodicIO();

  private final double m_turningOffset;

  private static class PeriodicIO {
    SwerveModuleState desiredState = new SwerveModuleState();
    // boolean shouldChangeState = false;
  }

  private boolean m_moduleDisabled = false;

  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel, double turningOffset) {
    m_turningOffset = turningOffset;

    m_driveMotor = new TalonFX(driveMotorChannel);
    m_driveMotor.setNeutralMode(NeutralModeValue.Coast);
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    driveConfig.Feedback.SensorToMechanismRatio = Constants.SwerveDrive.Drive.k_driveGearRatio;
    // m_driveConfiguration.Feedback.RotorToSensorRatio = 0.0f; TODO: DO THIS PLEASE GOD I HOPE

    driveConfig.Slot0.kP = Constants.SwerveDrive.Drive.k_P;
    driveConfig.Slot0.kI = Constants.SwerveDrive.Drive.k_I;
    driveConfig.Slot0.kD = Constants.SwerveDrive.Drive.k_D;
    driveConfig.Slot0.kS = Constants.SwerveDrive.Drive.k_S;
    driveConfig.Slot0.kV = Constants.SwerveDrive.Drive.k_V;
    driveConfig.Slot0.kA = Constants.SwerveDrive.Drive.k_A;
    // m_driveMotor.setSmartCurrentLimit(Constants.Drivetrain.Drive.k_driveCurrentLimit);

    TalonFXConfigurator driveConfigurator = m_driveMotor.getConfigurator();
    driveConfigurator.apply(driveConfig);

    m_turnMotor = new RARSparkMax(turningMotorChannel, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    config.inverted(true);

    config.encoder.positionConversionFactor(Constants.SwerveDrive.Turn.k_gearRatio * 2.0 * Math.PI);
    config.encoder.velocityConversionFactor(Constants.SwerveDrive.Turn.k_gearRatio * 2.0 * Math.PI / 60.0);

    config.closedLoop.p(Constants.SwerveDrive.Turn.k_P);
    config.closedLoop.i(Constants.SwerveDrive.Turn.k_I);
    config.closedLoop.d(Constants.SwerveDrive.Turn.k_D);
    config.closedLoop.positionWrappingEnabled(true);
    config.closedLoop.positionWrappingMinInput(0.0);
    config.closedLoop.positionWrappingMaxInput(2.0 * Math.PI);
    config.closedLoop.outputRange(
      Constants.SwerveDrive.Turn.k_turningMinOutput,
      Constants.SwerveDrive.Turn.k_turningMaxOutput);
    
    // m_turnMotor.setSmartCurrentLimit(Constants.SwerveDrive.Drive.k_turnCurrentLimit);
    // m_turningAbsEncoder = new TalonSRXMagEncoder(turningEncoderChannel);

    m_turnMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_turningPIDController = m_turnMotor.getClosedLoopController();
    m_turningAbsEncoder = m_turnMotor.getAbsoluteEncoder();
    m_turningRelEncoder = m_turnMotor.getEncoder();
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
    desiredState.optimize(Rotation2d.fromRadians(getTurnPosition()));
    desiredState.angle = new Rotation2d(Helpers.modRadians(desiredState.angle.getRadians()));
    // m_periodicIO.shouldChangeState = !desiredState.equals(m_periodicIO.desiredState);
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
    m_periodicIO.desiredState.optimize(Rotation2d.fromRadians(getTurnPosition()));
    // m_periodicIO.shouldChangeState = true;
  }

  public void periodic() {
    // if (m_periodicIO.shouldChangeState) {
    // if (!m_moduleDisabled) {
    double wheelCirc = Constants.SwerveDrive.k_wheelRadiusIn * 2.0d * Math.PI; // TODO: Move this

    VelocityVoltage request = new VelocityVoltage(getDriveTargetVelocity() / wheelCirc).withSlot(0);
    // PositionVoltage request = new PositionVoltage(0).withSlot(0);
    // request.Velocity = getDriveTargetVelocity() / wheelCirc;
    m_driveMotor.setControl(request);
    m_turningPIDController.setReference(getTurnTargetAngleRadians(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    
    // } else {
    //   DriverStation.reportWarning(m_moduleName + " is disabled, encoder is probably not plugged in!", false);
    //   m_driveMotor.setNeutralMode(NeutralModeValue.Coast);
      // m_turnMotor.setIdleMode(IdleMode.kCoast);

    //   // m_drivePIDController.setReference(0, ControlType.kVoltage);
    //   m_turningPIDController.setReference(0, ControlType.kVoltage);
    // }

    // m_periodicIO.shouldChangeState = false;
    // }
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
    return m_turningAbsEncoder.getPosition() - m_turningOffset; // TODO: verify this is the absolute position
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
