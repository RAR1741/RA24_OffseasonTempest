package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.RobotTelemetry;
import frc.robot.subsystems.Subsystem;

public class RAROdometry extends Subsystem {
  private static RAROdometry m_instance;
  private final SwerveDrive m_swerve;
  private final AHRS m_gyro;

  private SwerveDrivePoseEstimator m_poseEstimator;

  private RAROdometry() {
    super("Odometry");

    m_gyro = new AHRS(SPI.Port.kMXP);
    m_swerve = SwerveDrive.getInstance();

    m_poseEstimator = new SwerveDrivePoseEstimator(
      m_swerve.getKinematics(),
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_swerve.getModule(SwerveDrive.Module.FRONT_LEFT).getPosition(),
          m_swerve.getModule(SwerveDrive.Module.FRONT_RIGHT).getPosition(),
          m_swerve.getModule(SwerveDrive.Module.BACK_LEFT).getPosition(),
          m_swerve.getModule(SwerveDrive.Module.BACK_RIGHT).getPosition()
      },
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)) // TODO: CLARIFY THIS WORKS
  );
  }

  public static RAROdometry getInstance() {
    if(m_instance == null) {
      m_instance = new RAROdometry();
    }
    return m_instance;
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

  public Rotation2d getRotation2d() {
    return m_gyro.getRotation2d();
  }

  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_swerve.getModule(SwerveDrive.Module.FRONT_LEFT).getPosition(),
          m_swerve.getModule(SwerveDrive.Module.FRONT_RIGHT).getPosition(),
          m_swerve.getModule(SwerveDrive.Module.BACK_LEFT).getPosition(),
          m_swerve.getModule(SwerveDrive.Module.BACK_RIGHT).getPosition()
        },
        pose);
  }

  public void setGyroAngleAdjustment(double angle) {
    m_gyro.setAngleAdjustment(angle);
  }

  public void resetOdometry(Pose2d pose) {
    m_swerve.resetDriveEncoders();

    // We're manually setting the drive encoder positions to 0, since we
    // just reset them, but the encoder isn't reporting 0 yet.
    m_poseEstimator = new SwerveDrivePoseEstimator(
        m_swerve.getKinematics(),
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            new SwerveModulePosition(0.0,
                Rotation2d.fromRotations(m_swerve.getModule(SwerveDrive.Module.FRONT_LEFT).getTurnPosition())),
            new SwerveModulePosition(0.0,
                Rotation2d.fromRotations(m_swerve.getModule(SwerveDrive.Module.FRONT_RIGHT).getTurnPosition())),
            new SwerveModulePosition(0.0,
                Rotation2d.fromRotations(m_swerve.getModule(SwerveDrive.Module.BACK_LEFT).getTurnPosition())),
            new SwerveModulePosition(0.0,
                Rotation2d.fromRotations(m_swerve.getModule(SwerveDrive.Module.BACK_RIGHT).getTurnPosition())),
        },
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    setPose(pose);
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  @Override
  public void reset() {
    resetGyro();
    resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }

  @Override
  public void periodic() {
  }

  @Override
  public void writePeriodicOutputs() {
  }

  @Override
  public void stop() {
    RobotTelemetry.print("Stopping Odometry!");
  }

  private enum LimelightInstance {
    LEFT, RIGHT, CENTER
  }

}
