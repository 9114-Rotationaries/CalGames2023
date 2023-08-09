// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {

    public static final Translation2d[] moduleTranslations = {new Translation2d(0.381, 0.381), new Translation2d(0.381, -0.381),
    new Translation2d(-0.381, 0.381), new Translation2d(-0.381, -0.381)};
    
    public static final int pigeon = 9;
    public static final double kMaxSpeed = 4; // 1 meters per second
    public static final double kMaxAngularSpeed = 12; // 12 radians? per second

    //FL-Front Left, DM-Driving Motor, TM-Turning Motor, TE-Turning Encoder
    public static final int FLDMChannel = 12;
    public static final int FLTMChannel = 11;
    public static final int FLTEChannel = 13;
    public static final double FLTEOffsetDegrees = 278.174;

    public static final int FRDMChannel = 3;
    public static final int FRTMChannel = 2;
    public static final int FRTEChannel = 4;
    public static final double FRTEOffsetDegrees = 343.916;

    public static final int BLDMChannel = 9;
    public static final int BLTMChannel = 8;
    public static final int BLTEChannel = 10;
    public static final double BLTEOffsetDegrees = 119.707;

    public static final int BRDMChannel = 6;
    public static final int BRTMChannel = 5;
    public static final int BRTEChannel = 7;
    public static final double BRTEOffsetDegrees = 181.494;
  }
  
  public static class SwerveConstants {
    public static final double kWheelRadius = 0.0508;
    public static final int kEncoderResolution = 42;

    public static final double kModuleMaxAngularVelocity = DriveConstants.kMaxAngularSpeed;
    public static final double kModuleMaxAngularAcceleration = 5; // radians per second squared

    // drive PID constants
    public static final double PIDp = 0.2;
    public static final double PIDi = 0;
    public static final double PIDd = 0.0;

    // turning PID constants 
    public static final double ProfiledPIDp = 0.4;
    public static final double ProfiledPIDi = 0.0000000;
    public static final double ProfiledPIDd = 0.0001;

    public static final double DriveKs = 0;
    public static final double DriveKv = 0;

    public static final double TurnKs = 0;
    public static final double TurnKv = 0;
  }

  public static class VisionConstants{
    public static final double targetHeight = 0;
    public static final double cameraHeight = 0;
    public static final double cameraAngle = 0;

    //MoveToTag Angle PID Constants
    public static final double VAnglePIDp = 0.1;
    public static final double VAnglePIDi = 0;
    public static final double VAnglePIDd = 0;

    //MoveToTag Drive PID Constants
    public static final double VDrivePIDp = 0.1;
    public static final double VDrivePIDi = 0;
    public static final double VDrivePIDd = 0;

    //MoveToTag Rot PID Constants
    public static final double VRotPIDp = 0.1;
    public static final double VRotPIDi = 0;
    public static final double VRotPIDd = 0;

    //Tag Setpoints
    public static final double alignSetpoint = 0; //Center
    public static final double driveSetpoint = 0; //Change to how far away from tag desired
    public static final double rotSetpoint = 0; //Oriented

    //Tolerances
    public static final double angleTolerance = 0.02;
    public static final double distanceTolerance = 0.02;
    public static final double rotTolerance = 0.02;
  }

}