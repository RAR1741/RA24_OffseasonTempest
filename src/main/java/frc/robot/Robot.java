package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.AutoChooser;
import frc.robot.autonomous.AutoRunner;
import frc.robot.autonomous.tasks.Task;
import frc.robot.controls.controllers.DriverController;
import frc.robot.controls.controllers.FilteredController;
import frc.robot.controls.controllers.OperatorController;
import frc.robot.simulation.Field;
import frc.robot.subsystems.Subsystem;

public class Robot extends LoggedRobot {
  private final DriverController m_driverController = new DriverController(0, true, true);
  private final OperatorController m_operatorController = new OperatorController(1, true, true);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xRateLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yRateLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotRateLimiter = new SlewRateLimiter(3);

  // Robot subsystems
  private List<Subsystem> m_allSubsystems = new ArrayList<>();
  private List<FilteredController> m_allControllers = new ArrayList<>();
  // public final SwerveDrive m_swerve = new SwerveDrive();
  private Task m_currentTask;
  private AutoRunner m_autoRunner = AutoRunner.getInstance();

  // The mere instantiation of this object will cause the compressor to start
  // running. We don't need to do anything else with it, so we'll suppress the
  // warning.
  @SuppressWarnings("unused")
  private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);

  @SuppressWarnings("unused")
  private UsbCamera m_camera;

  // Auto things
  AutoChooser m_autoChooser = new AutoChooser();

  private final Field m_field = Field.getInstance();

  @Override
  public void robotInit() {
    new RobotTelemetry();

    // Initialize on-board logging
    DataLogManager.start();
    System.out.println("Logging initialized. Fard.");

    // Set up demo mode picker
    if (!Preferences.containsKey("demoMode")) {
      Preferences.setBoolean("demoMode", false);
    }
    if (!Preferences.containsKey("demoLEDMode")) {
      Preferences.setInt("demoLEDMode", 0);
    }

    // Set up the Field2d object for simulation
    SmartDashboard.putData("Field", m_field);

    // Camera server
    m_camera = CameraServer.startAutomaticCapture();

    // m_allSubsystems.add(m_swerve);

    m_allControllers.add(m_driverController);
    // m_allControllers.add(m_operatorController);
  }

  @Override
  public void robotPeriodic() {
    m_allControllers.forEach(controller -> controller.periodic());
    m_allSubsystems.forEach(subsystem -> subsystem.periodic());
    m_allSubsystems.forEach(subsystem -> subsystem.writePeriodicOutputs());
    m_allSubsystems.forEach(subsystem -> subsystem.writeToLog());

    updateSim();
  }

  @Override
  public void autonomousInit() {
    // m_swerve.brakeOff();

    m_autoRunner.setAutoMode(m_autoChooser.getSelectedAuto());
    m_currentTask = m_autoRunner.getNextTask();

    // Start the first task
    if (m_currentTask != null) {
      m_currentTask.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    // If there is a current task, run it
    if (m_currentTask != null) {
      // Run the current task
      m_currentTask.update();
      m_currentTask.updateSim();

      // If the current task is finished, get the next task
      if (m_currentTask.isFinished()) {
        m_currentTask.done();
        m_currentTask = m_autoRunner.getNextTask();

        // Start the next task
        if (m_currentTask != null) {
          m_currentTask.start();
        }
      }
    }
  }

  @Override
  public void teleopInit() {
    // m_swerve.brakeOff();
    // m_swerve.drive(0, 0, 0, false);
    // m_swerve.setGyroAngleAdjustment(0);
  }

  @Override
  public void teleopPeriodic() {
    double xSpeed = m_xRateLimiter.calculate(m_driverController.getForwardAxis());
    double ySpeed = m_yRateLimiter.calculate(m_driverController.getStrafeAxis());

    double rot = m_rotRateLimiter.calculate(m_driverController.getTurnAxis());

    // slowScaler should scale between k_slowScaler and 1
    double slowScaler = Constants.SwerveDrive.k_slowScaler
        + ((1 - m_driverController.getSlowScaler()) * (1 - Constants.SwerveDrive.k_slowScaler));

    // boostScaler should scale between 1 and k_boostScaler
    double boostScaler = 1 + (m_driverController.getBoostScaler() * (Constants.SwerveDrive.k_boostScaler - 1));

    if (Preferences.getBoolean("demoMode", false)) {
      // boostScaler = 1;
      xSpeed *= Constants.SwerveDrive.k_maxDemoSpeed;
      ySpeed *= Constants.SwerveDrive.k_maxDemoSpeed;
      rot *= Constants.SwerveDrive.k_maxDemoAngularSpeed;
    } else {
      xSpeed *= Constants.SwerveDrive.k_maxSpeed;
      ySpeed *= Constants.SwerveDrive.k_maxSpeed;
      rot *= Constants.SwerveDrive.k_maxAngularSpeed;
    }

    xSpeed *= slowScaler * boostScaler;
    ySpeed *= slowScaler;// * boostScaler;
    if (Preferences.getBoolean("demoMode", false)) {
      ySpeed *= boostScaler;
    }
    rot *= slowScaler * boostScaler;

    // m_swerve.drive(xSpeed, ySpeed, rot, true);

    if (m_driverController.getWantsResetGyro()) {
      // m_swerve.resetGyro();
    }
  }

  @Override
  public void simulationPeriodic() {
    updateSim();
  }

  @Override
  public void disabledInit() {
    m_allSubsystems.forEach(subsystem -> subsystem.stop());
    // m_swerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void disabledPeriodic() {
    // m_allSubsystems.forEach(subsystem -> subsystem.outputTelemetry());

    updateSim();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    // m_swerve.drive(0, 0, 0, false);
  }

  private void updateSim() {
    // Update the odometry in the sim.
    // m_field.setRobotPose(m_swerve.getPose());
  }
}
