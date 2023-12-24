package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

  public static class RobotConstants{
    public static enum RobotType {
      ROBOT_2023Cal,
      ROBOT_SIMBOT
    }

    private static final RobotType robot = RobotType.ROBOT_SIMBOT;

    public static RobotType getRobot() {
      if (!disableHAL && RobotBase.isReal()) {
        if (robot == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
          return RobotType.ROBOT_2023Cal;
        } else {
          return robot;
        }
      } else {
        return robot;
      }
    }

    public static boolean disableHAL = false;
    public static void disableHAL() {
      disableHAL = true;
    }
}
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class IntakeConstants {
    public static final int cubeIntakeChannel = 60;
    public static final int coneIntakeChannel = 61;

    public static final double cubeIntakeSpeed = -0.54;
    public static final double coneIntakeSpeed = .6;

    public static final double cubeOuttakeSpeed = 0.84;
    public static final double autoCubeOuttakeSpeed = 0.4;
    public static final double coneOuttakeSpeed = -1;

    public static final double cubeLaunchSpeed = -1;
    public static final double coneLaunchSpeed = -1;
  }

  public static class ArmConstants {
    public static final int rightArmChannel = 50;
    public static final int leftArmChannel = 51;
    public static final double pivotSpeed = -0.9; //pivot speed based on right motor
    public static final double armRatio = 108;

    public static final double armPIDp = 0.0;
    public static final double armPIDi = 0.0;
    public static final double armPIDd = 0.0;

    public static final double armTolerance = 0.0;
  }

  public static class DriveConstants {

    public static final Translation2d m_frontLeftLocation = new Translation2d(0.3429, 0.3429); //meters
    public static final Translation2d m_frontRightLocation = new Translation2d(0.3429, -0.3429);
    public static final Translation2d m_backLeftLocation = new Translation2d(-0.3429, 0.3429);
    public static final Translation2d m_backRightLocation = new Translation2d(-0.3429, -0.3429);
    

    public static final double kCommunitySpeed = 7;
    public static final double kCommunityAngularSpeed = 7;
    public static final double kMaxSpeed = 4; // 1 meters per second
    public static final double kMaxAngularSpeed = 3; // 12 radians? per second

    //FL-Front Left, DM-Driving Motor, TM-Turning Motor, TE-Turning Encoder

    public static final int FRDMChannel = 10;
    public static final int FRTMChannel = 11;
    public static final int FRTEChannel = 13;
    public static final double FRTEOffsetDegrees = 343.916;

    public static final int FLDMChannel = 20;
    public static final int FLTMChannel = 21;
    public static final int FLTEChannel = 23;
    public static final double FLTEOffsetDegrees = 278.174;

    public static final int BLDMChannel = 30;
    public static final int BLTMChannel = 31;
    public static final int BLTEChannel = 33;
    public static final double BLTEOffsetDegrees = 119.707;

    public static final int BRDMChannel = 40;
    public static final int BRTMChannel = 41;
    public static final int BRTEChannel = 43;
    public static final double BRTEOffsetDegrees = 181.494;
  }
  
  public static class SwerveConstants {
    public static final double kWheelRadius = 0.0508;
    public static final int kEncoderResolution = 42;

    public static final double kModuleMaxAngularVelocity = DriveConstants.kMaxAngularSpeed;
    public static final double kModuleMaxAngularAcceleration = 50; // radians per second squared

    // drive PID constants
    public static final double PIDp = 0.225;
    public static final double PIDi = 0;
    public static final double PIDd = 0.004225;

    // turning PID constants 
    public static final double turningPIDp = 0.6; //.6
    public static final double turningPIDi = 0;
    public static final double turningPIDd = 0.02; //.02

    public static final double DriveKs = 0;
    public static final double DriveKv = 0;

    public static final double TurnKs = 0;
    public static final double TurnKv = 0;
  }


  public static class VisionConstants{
    public static final double targetHeight = 0.5969;
    public static final double cameraHeight = 0.2286;  
    public static final double cameraAngle = 5;

    //MoveToTag Angle PID Constants
    public static final double VAnglePIDp = 0.555;
    public static final double VAnglePIDi = 0;
    public static final double VAnglePIDd = 0.003;

    //MoveToTag Drive PID Constants
    public static final double VDrivePIDp = 0.555;
    public static final double VDrivePIDi = 0;
    public static final double VDrivePIDd = 0.003;

    //MoveToTag Rot PID Constants
    public static final double ProfiledVRotPIDp = 0.8;
    public static final double ProfiledVRotPIDi = 0;
    public static final double ProfiledVRotPIDd = 0.0002;

    //Tag Setpoints
    public static final double alignSetpoint = 0; //Center
    public static final double driveSetpoint = 29; //Change to how far away from tag desired
    public static final double rotSetpoint = 0; //Oriented

    //Tolerances
    public static final double angleTolerance = 0.02;
    public static final double distanceTolerance = 0.02;
    public static final double rotTolerance = 0.02;
  }

  public static class AutoConstants {
    public static final double maxVelocity = 4.5;
    public static final double maxAcceleration = 3;

    // drive PID constants
    public static final double PIDp = 0.225;
    public static final double PIDi = 0;
    public static final double PIDd = 0.004225;

    // turning PID constants 
    public static final double ProfiledPIDp = 0.8;
    public static final double ProfiledPIDi = 0;
    public static final double ProfiledPIDd = 0.02;
  }

  public static class SimConstants {
    public static final double loopPeriodSecs = 0.02;

    public static final Translation2d m_frontLeftLocation = new Translation2d(0.3429, 0.3429); //meters
    public static final Translation2d m_frontRightLocation = new Translation2d(0.3429, -0.3429);
    public static final Translation2d m_backLeftLocation = new Translation2d(-0.3429, 0.3429);
    public static final Translation2d m_backRightLocation = new Translation2d(-0.3429, -0.3429);

    public static final double DrivePIDp = 0.0;
    public static final double DrivePIDi = 0.0;
    public static final double DrivePIDd = 0.0;

    public static final double TurnPIDp = 0.0;
    public static final double TurnPIDi = 0.0;
    public static final double TurnPIDd = 0.0;

    public static final double kMaxSpeed = 4;
    public static final double kMaxAngularSpeed = 3;

  }

}
