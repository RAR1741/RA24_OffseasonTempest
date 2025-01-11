package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class Robot {
    public static final double k_width = 27; // Inches
    public static final double k_length = 30; // Inches

    public static final double k_bumperStart = 1; // Inches
    public static final double k_bumperHeight = 5; // Inches
  }

  public static class Simulation {
    public static final double k_width = 150; // Inches
    public static final double k_height = 80; // Inches
  }

  public static class Auto {
    public static final double k_maxSpeed = 1; // 1 meters per second
    public static final double k_maxAcceleration = 0.5;
    public static final double k_defaultGripperWait = 0.5;
  }

  public class SwerveDrive {
    // Drivetrain wheel offsets
    // TODO: Change for final robot
    public static final double k_xDistance = 0.762; // 30 inches Forward/Backward
    public static final double k_yDistance = 0.6858; // in meters! Side-to-Side

    public static final double k_xCenterDistance = k_xDistance / 2.0;
    public static final double k_yCenterDistance = k_yDistance / 2.0;
    public static final double k_wheelRadiusIn = 2; // 2 inches

    public static final double k_maxSpeed = 3.0; // 3 meters per second
    public static final double k_maxAngularSpeed = Math.PI; // 1/2 rotation per second
    public static final double k_slowScaler = 0.2; // 20% reduction in speed
    public static final double k_boostScaler = 2.0; // 200% increase in speed

    public static final double k_maxDemoSpeed = 1;
    public static final double k_maxDemoAngularSpeed = Math.PI / 2;

    // Drivetrain drive motor constants
    public class Drive {
      public static final int k_FLMotorId = 5;
      public static final int k_FRMotorId = 6;
      public static final int k_BLMotorId = 7;
      public static final int k_BRMotorId = 8;

      // These values were obtained via SysId
      // In "Feedback Analysis" section, we set Measurement Delay to 58 ms, which was
      // pulled from the "Feedforward Analysis" Velocity Measurement Delay
      public static final double k_P = 2.0;
      public static final double k_I = 0.0;
      public static final double k_D = 0.0;

      public static final double k_S = 0.0; //0.25043;
      public static final double k_V = 0.0; //3.0125;
      public static final double k_A = 0.0; //0.38005;

      public static final double k_driveGearRatio = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0); //(50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
      public static final double k_driveEncPerRot = 2048.0;
      public static final double k_driveEncPerSec = 2048.0 / 10.0; // Encoder reports 2048 Encoder counter per 100 ms
    }

    // Drivetrain (turn) constants
    public class Turn {
      // Drivetrain turning offset constants
      public static final double k_FLOffset = 0.796705;
      public static final double k_FROffset = 0.884635;
      public static final double k_BLOffset = 0.754982;
      public static final double k_BROffset = 0.103446;

      public static final double k_gearRatio = 7.0 / 150.0;

      public static final double k_P = 2.0;
      public static final double k_I = 0.0;
      public static final double k_D = 0.0;

      public static final double k_S = 0.0;
      public static final double k_V = 0.0;
      public static final double k_A = 0.0;

      public static final int k_FLMotorId = 9;
      public static final int k_FRMotorId = 10;
      public static final int k_BLMotorId = 11;
      public static final int k_BRMotorId = 12;

      public static final int k_FLAbsId = 99;
      public static final int k_FRAbsId = 99;
      public static final int k_BLAbsId = 99;
      public static final int k_BRAbsId = 99;

      public static final double k_turningMaxOutput = 1.0;
      public static final double k_turningMinOutput = -1.0;
    }
  }

  public static class Field {
    // All dimensions from Figure 5-16 in the manual
    public static final double k_lowGoalX = 22.75; // Inches
    public static final double k_lowGoalHeight = 34; // Inches

    public static final double k_highGoalX = 39.75; // Inches
    public static final double k_highGoalHeight = 46; // Inches

    public static final double k_width = Units.feetToMeters(54.0);
    public static final double k_length = Units.feetToMeters(27.0);
  }
}
