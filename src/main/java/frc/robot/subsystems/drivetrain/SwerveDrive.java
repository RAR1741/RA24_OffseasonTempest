package frc.robot.subsystems.drivetrain;

import java.util.ArrayList;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;

public class SwerveDrive extends Subsystem {
  public SwerveDrive() {
    super("SwerveDrive");
  }

  private static SwerveDrive m_swerve = null;

  private final RAROdometry m_odometry = RAROdometry.getInstance();
  private final AHRS m_gyro = m_odometry.getGyro();

  // Robot "forward" is +x
  // Robot "left" is +y
  // Robot "clockwise" is -z
  private final Translation2d m_frontLeftLocation = new Translation2d(Constants.SwerveDrive.k_xCenterDistance,
      Constants.SwerveDrive.k_yCenterDistance);
  private final Translation2d m_frontRightLocation = new Translation2d(Constants.SwerveDrive.k_xCenterDistance,
      -Constants.SwerveDrive.k_yCenterDistance);
  private final Translation2d m_backLeftLocation = new Translation2d(-Constants.SwerveDrive.k_xCenterDistance,
      Constants.SwerveDrive.k_yCenterDistance);
  private final Translation2d m_backRightLocation = new Translation2d(-Constants.SwerveDrive.k_xCenterDistance,
      -Constants.SwerveDrive.k_yCenterDistance);

  private static final SwerveModule[] m_modules = {
      new SwerveModule(Constants.SwerveDrive.Drive.k_FLMotorId,
          Constants.SwerveDrive.Turn.k_FLMotorId,
          Constants.SwerveDrive.Turn.k_FLOffset, "FL"), // 0
      new SwerveModule(Constants.SwerveDrive.Drive.k_FRMotorId,
          Constants.SwerveDrive.Turn.k_FRMotorId,
          Constants.SwerveDrive.Turn.k_FROffset, "FR"), // 1
      new SwerveModule(Constants.SwerveDrive.Drive.k_BRMotorId,
          Constants.SwerveDrive.Turn.k_BRMotorId,
          Constants.SwerveDrive.Turn.k_BROffset, "BR"), // 2
      new SwerveModule(Constants.SwerveDrive.Drive.k_BLMotorId,
          Constants.SwerveDrive.Turn.k_BLMotorId,
          Constants.SwerveDrive.Turn.k_BLOffset, "BL") // 3
  };

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  public static SwerveDrive getInstance() {
    if (m_swerve == null) {
      m_swerve = new SwerveDrive();
    }
    return m_swerve;
  }

  public void brakeOn() {
    for (SwerveModule module : m_modules) {
      module.getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
    }
  }

  public void brakeOff() {
    for (SwerveModule module : m_modules) {
      module.getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
    }
  }

  public void clearTurnPIDAccumulation() {
    for (SwerveModule module : m_modules) {
      module.clearTurnPIDAccumulation();
    }
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    double maxBoostSpeed = Constants.SwerveDrive.k_maxSpeed * Constants.SwerveDrive.k_boostScaler;
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxBoostSpeed);

    for (int i = 0; i < 3; i++) {
      m_modules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  public void pointModules(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    // Zero out the speed component of each swerve module state
    for (SwerveModuleState moduleState : swerveModuleStates) {
      moduleState.speedMetersPerSecond = 0.0;
    }

    for (int i = 0; i < 3; i++) {
      m_modules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  public void resetDriveEncoders() {
    for (SwerveModule module : m_modules) {
      module.resetDriveEncoder();
    }
  }

  @Override
  public void periodic() {
    for (SwerveModule module : m_modules) {
      module.periodic();
    }
  }

  @Override
  public void stop() {
    brakeOn();
    drive(0.0, 0.0, 0.0, true);
  }

  @Override
  public void writePeriodicOutputs() {
  }

  private ArrayList<Double> getCurrentStates() {
    ArrayList<Double> currentStates = new ArrayList<>();

    for (SwerveModule module : m_modules) {
      currentStates.add(module.getTurnPosition() * 360);
      currentStates.add(module.getDriveVelocity());
    }

    return currentStates;
  }

  private ArrayList<Double> getDesiredStates() {
    ArrayList<Double> desiredStates = new ArrayList<>();

    for (SwerveModule module : m_modules) {
      desiredStates.add(module.getDesiredState().angle.getDegrees());
      desiredStates.add(module.getDesiredState().speedMetersPerSecond);
    }

    return desiredStates;
  }

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public SwerveModule getModule(int module) {
    switch(module) {
      case Module.FRONT_LEFT -> {
        return m_modules[Module.FRONT_LEFT];
      }
      case Module.FRONT_RIGHT -> {
        return m_modules[Module.FRONT_LEFT];
      }
      case Module.BACK_LEFT -> {
        return m_modules[Module.FRONT_LEFT];
      }
      case Module.BACK_RIGHT -> {
        return m_modules[Module.FRONT_LEFT];
      }
    }
    return null;
  }

  // @Override
  // public void outputTelemetry() {
  // double currentTime = Timer.getFPGATimestamp();

  // m_poseEstimator.updateWithTime(
  // currentTime,
  // m_gyro.getRotation2d(),
  // new SwerveModulePosition[] {
  // m_frontLeft.getPosition(),
  // m_frontRight.getPosition(),
  // m_backLeft.getPosition(),
  // m_backRight.getPosition()
  // });

  // // if (m_limelight.seesAprilTag()) {
  // // m_poseEstimator.addVisionMeasurement(
  // // m_limelight.getBotpose2D(),
  // // m_limelight.getTimeOffset(currentTime));
  // // }

  // m_frontLeft.outputTelemetry();
  // m_frontRight.outputTelemetry();
  // m_backLeft.outputTelemetry();
  // m_backRight.outputTelemetry();

  // SmartDashboard.putNumberArray("Drivetrain/CurrentStates",
  // getCurrentStates());
  // SmartDashboard.putNumberArray("Drivetrain/DesiredStates",
  // getDesiredStates());

  // SmartDashboard.putNumber("Drivetrain/Gyro/AngleDegrees",
  // m_gyro.getRotation2d().getDegrees());
  // SmartDashboard.putNumber("Drivetrain/Gyro/Pitch", m_gyro.getPitch());
  // SmartDashboard.putNumberArray("Drivetrain/Pose",
  // new double[] { getPose().getX(), getPose().getY(),
  // getPose().getRotation().getDegrees() });
  // }

  public interface Module {
    int FRONT_LEFT = 0;
    int FRONT_RIGHT = 1;
    int BACK_RIGHT = 2;
    int BACK_LEFT = 3;
  }

  @Override
  public void reset() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }
}
