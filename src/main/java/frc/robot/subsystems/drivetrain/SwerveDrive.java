package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;

public class SwerveDrive extends Subsystem {
  private static SwerveDrive m_swerve = null;
  
  private SwerveDrive() {
    super("SwerveDrive");
  }

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
          Constants.SwerveDrive.Turn.k_FLAbsId,
          Constants.SwerveDrive.Turn.k_FLOffset), // 0
      new SwerveModule(Constants.SwerveDrive.Drive.k_FRMotorId,
          Constants.SwerveDrive.Turn.k_FRMotorId,
          Constants.SwerveDrive.Turn.k_FRAbsId,
          Constants.SwerveDrive.Turn.k_FROffset), // 1
      new SwerveModule(Constants.SwerveDrive.Drive.k_BRMotorId,
          Constants.SwerveDrive.Turn.k_BRMotorId,
          Constants.SwerveDrive.Turn.k_BRAbsId,
          Constants.SwerveDrive.Turn.k_BROffset), // 2
      new SwerveModule(Constants.SwerveDrive.Drive.k_BLMotorId,
          Constants.SwerveDrive.Turn.k_BLMotorId,
          Constants.SwerveDrive.Turn.k_BLAbsId,
          Constants.SwerveDrive.Turn.k_BLOffset) // 3
  };

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  public static SwerveDrive getInstance() {
    if (m_swerve == null) {
      m_swerve = new SwerveDrive();
    }
    return m_swerve;
  }

  public void setBrake(boolean isBrake) {
    for (SwerveModule module : m_modules) {
      if(isBrake) {
        module.getDriveMotor().setNeutralMode(NeutralModeValue.Brake);
      } else {
        module.getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
      }
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
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, RAROdometry.getInstance().getGyro().getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    double maxBoostSpeed = Constants.SwerveDrive.k_maxSpeed * Constants.SwerveDrive.k_boostScaler;
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxBoostSpeed);

    for (int i = 0; i < m_modules.length; i++) {
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
    setBrake(true);
    drive(0.0, 0.0, 0.0, true);
  }

  @Override
  public void writePeriodicOutputs() {
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

  // Logged
  // @AutoLogOutput
  // public Rotation2d getRotation2d() {
  //   return m_poseEstimator.getEstimatedPosition().getRotation();
  // }

  @AutoLogOutput
  private SwerveModuleState[] getCurrentStates() {
    SwerveModuleState[] currentStates = {
        m_modules[Module.FRONT_LEFT].getState(),
        m_modules[Module.FRONT_RIGHT].getState(),
        m_modules[Module.BACK_RIGHT].getState(),
        m_modules[Module.BACK_LEFT].getState()
    };

    return currentStates;
  }

  @AutoLogOutput
  private SwerveModuleState[] getDesiredStates() {
    SwerveModuleState[] desiredStates = {
        m_modules[Module.FRONT_LEFT].getDesiredState(),
        m_modules[Module.FRONT_RIGHT].getDesiredState(),
        m_modules[Module.BACK_RIGHT].getDesiredState(),
        m_modules[Module.BACK_LEFT].getDesiredState()
    };

    return desiredStates;
  }

  // @AutoLogOutput
  // public boolean hasSetPose() {
  //   return m_hasSetPose;
  // }

  // @AutoLogOutput
  // private double getGyroYaw() {
  //   return m_gyro.getRotation2d().getDegrees();
  // }

  // @AutoLogOutput
  // private double getGyroPitch() {
  //   return m_gyro.getPitch();
  // }

  // @AutoLogOutput
  // public Pose2d getPose() {
  //   return m_poseEstimator.getEstimatedPosition();
  // }

  // @AutoLogOutput
  // public double getNavXTimestamp() {
  //   return (double) m_gyro.getLastSensorTimestamp();
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
