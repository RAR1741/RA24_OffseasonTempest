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

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  // private final Limelight m_limelight = Limelight.getInstance();

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_modules[Module.FRONT_LEFT].getPosition(),
          m_modules[Module.FRONT_RIGHT].getPosition(),
          m_modules[Module.BACK_LEFT].getPosition(),
          m_modules[Module.BACK_RIGHT].getPosition()
      },
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)) // TODO: CLARIFY THIS WORKS
  );

  // private SwerveDrivePoseEstimator poseEstimator2 = new SwerveDrivePoseEstimator(m_kin)

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

  public void reset() {
    resetGyro();
    resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }

  /**
   * Calls the NavX reset function, resetting the Z angle to 0
   */
  public void resetGyro() {
    m_gyro.reset();
    m_gyro.setAngleAdjustment(0.0);
  }

  public AHRS getGyro() {
    return m_gyro;
  }

  public void setGyroAngleAdjustment(double angle) {
    m_gyro.setAngleAdjustment(angle);
  }

  public Rotation2d getRotation2d() {
    return m_gyro.getRotation2d();
  }

  public void clearTurnPIDAccumulation() {
    for (SwerveModule module : m_modules) {
      module.clearTurnPIDAccumulation();
    }
  }

  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_modules[Module.FRONT_LEFT].getPosition(),
            m_modules[Module.FRONT_RIGHT].getPosition(),
            m_modules[Module.BACK_LEFT].getPosition(),
            m_modules[Module.BACK_RIGHT].getPosition()
        },
        pose);
  }

  public void resetOdometry(Pose2d pose) {
    for (SwerveModule module : m_modules) {
      module.resetDriveEncoder();
    }

    // We're manually setting the drive encoder positions to 0, since we
    // just reset them, but the encoder isn't reporting 0 yet.
    m_poseEstimator = new SwerveDrivePoseEstimator(
        m_kinematics,
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            new SwerveModulePosition(0.0,
                Rotation2d.fromRotations(m_modules[Module.FRONT_LEFT].getTurnPosition())),
            new SwerveModulePosition(0.0,
                Rotation2d.fromRotations(m_modules[Module.FRONT_RIGHT].getTurnPosition())),
            new SwerveModulePosition(0.0,
                Rotation2d.fromRotations(m_modules[Module.BACK_LEFT].getTurnPosition())),
            new SwerveModulePosition(0.0,
                Rotation2d.fromRotations(m_modules[Module.BACK_RIGHT].getTurnPosition())),
        },
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    setPose(pose);
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

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
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
}
