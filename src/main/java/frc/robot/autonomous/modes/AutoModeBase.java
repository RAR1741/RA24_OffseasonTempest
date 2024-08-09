package frc.robot.autonomous.modes;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.autonomous.tasks.Task;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public abstract class AutoModeBase {
  private ArrayList<Task> m_tasks;

  public AutoModeBase() {
    m_tasks = new ArrayList<>();

    // Reset the gyro and set the starting position
    Pose2d startingPosition = getStartingPosition();
    SwerveDrive swerve = SwerveDrive.getInstance();
    swerve.setGyroAngleAdjustment(startingPosition.getRotation().getDegrees());
    swerve.resetOdometry(startingPosition);
  }

  public Task getNextTask() {
    // Pop the first task off the list and return it
    try {
      return m_tasks.remove(0);
    } catch (IndexOutOfBoundsException ex) {
      return null;
    }
  }

  public void queueTask(Task task) {
    m_tasks.add(task);
  }

  public abstract void queueTasks();

  public abstract Pose2d getRedStartingPosition();

  private Pose2d getBlueStartingPosition() {
    Rotation2d blueStartingRotation = Rotation2d.fromDegrees(getRedStartingPosition().getRotation().getDegrees() - 180);

    Translation2d blueStartingTranslation = new Translation2d(
        Constants.Field.k_width - getRedStartingPosition().getX(),
        getRedStartingPosition().getY());

    return new Pose2d(blueStartingTranslation, blueStartingRotation);
  };

  private Pose2d getStartingPosition() {
    return getRedStartingPosition();
  }
}
