// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kOperatorControllerPort = 1;
  }

  public static class IntakeConstants {
    public static final int cubeIntakeChannel = 60;
    public static final int coneIntakeChannel = 61;

    public static final double cubeIntakeSpeed = 0.5;
    public static final double coneIntakeSpeed = 0.5;

    public static final double cubeOuttakeSpeed = -0.5;
    public static final double coneOuttakeSpeed = -0.5;

    public static final double cubeLaunchSpeed = -1;
    public static final double coneLaunchSpeed = -1;
  }

  public static class ArmConstants {
    public static final int rightArmChannel = 50;
    public static final int leftArmChannel = 51;
    public static final double pivotSpeed = 0.5; //pivot speed based on right motor
    public static final double armRatio = 108;
  }
}