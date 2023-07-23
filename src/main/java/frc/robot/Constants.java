// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public static final double kMaxSpeed = 10.0; // 3 meters per second
    public static final double kMaxAngularSpeed = 6; // 1/2 rotation per second

    //FL-Front Left, DM-Driving Motor, TM-Turning Motor, TE-Turning Encoder
    public static final int FLDMChannel = 20;
    public static final int FLTMChannel = 21;
    public static final int FLDEChannelA = 22;
    public static final int FLDEChannelB = 23;
    public static final int FLTEChannelA = 24;
    public static final int FLTEChannelB = 25;
    //public static final double FLTEOffsetDegrees = 0;

    public static final int FRDMChannel = 3;
    public static final int FRTMChannel = 2;
    public static final int FRDEChannelA = 5;
    public static final int FRDEChannelB = 4;
    public static final int FRTEChannelA = 7;
    public static final int FRTEChannelB = 6;
    //public static final double FRTEOffsetDegrees = 0;

    public static final int BLDMChannel = 14;
    public static final int BLTMChannel = 15;
    public static final int BLDEChannelA = 16;
    public static final int BLDEChannelB = 17;
    public static final int BLTEChannelA = 18;
    public static final int BLTEChannelB = 19;
    //public static final double BLTEOffsetDegrees = 0;

    public static final int BRDMChannel = 10;
    public static final int BRTMChannel = 8;
    public static final int BRDEChannelA = 9;
    public static final int BRDEChannelB = 11;
    public static final int BRTEChannelA = 13;
    public static final int BRTEChannelB = 12;
    //public static final double BRTEOffsetDegrees = 0;
  }
  
  public static class ModuleConstants {
    public static final double kWheelRadius = 0.0508;
    public static final int kEncoderResolution = 4096;

    public static final double kModuleMaxAngularVelocity = DriveConstants.kMaxAngularSpeed;
    public static final double kModuleMaxAngularAcceleration = 10; // radians per second squared

    // drive PID constants
    public static final double PIDp = 0.1;
    public static final double PIDi = 0;
    public static final double PIDd = 0;

    // turning PID constants 
    public static final double ProfiledPIDp = 0.02;
    public static final double ProfiledPIDi = 0;
    public static final double ProfiledPIDd = 0.0001;

    public static final double DriveKs = 0;
    public static final double DriveKv = 0;

    public static final double TurnKs = 0;
    public static final double TurnKv = 0;
  }
}