// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

public class Drivetrain extends SubsystemBase{
  
  private final SwerveModule m_frontLeft = new SwerveModule(
    DriveConstants.FLDMChannel, DriveConstants.FLTMChannel, 
    DriveConstants.FLTEChannel, DriveConstants.FLTEOffsetDegrees);
  private final SwerveModule m_frontRight = new SwerveModule(
    DriveConstants.FRDMChannel, DriveConstants.FRTMChannel, 
    DriveConstants.FRTEChannel, DriveConstants.FRTEOffsetDegrees);
  private final SwerveModule m_backLeft = new SwerveModule(
    DriveConstants.BLDMChannel, DriveConstants.BLTMChannel, 
    DriveConstants.BLTEChannel, DriveConstants.BLTEOffsetDegrees);
  private final SwerveModule m_backRight = new SwerveModule(
    DriveConstants.BRDMChannel, DriveConstants.BRTMChannel, 
    DriveConstants.BRTEChannel, DriveConstants.BRTEOffsetDegrees);
  

  private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          DriveConstants.m_frontLeftLocation, DriveConstants.m_frontRightLocation, DriveConstants.m_backLeftLocation, DriveConstants.m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          ahrs.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    ahrs.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, ahrs.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    
    SmartDashboard.putNumber("Encodercounts", getEncoderCounts());
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        ahrs.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  @Override
  public void periodic(){}
    
  // public void resetPose(Pose2d pose) {
  //   m_odometry.resetPosition(ahrs.getRotation2d(), 
  //   new SwerveModulePosition[] {
  //     m_frontLeft.setPosition(0),
  //     m_frontRight.setPosition(0),
  //     m_backLeft.setPosition(0),
  //     m_backRight.setPosition(0)
  //   }, pose);
  // }

  public SwerveModule getModule() {
    return m_frontLeft;
  }

  public double getDistancePerPulse() {
    // Return the conversion factor for distance per pulse
    return 4096 *6.75 /((2 * Math.PI * SwerveConstants.kWheelRadius / 1)*(2 * Math.PI * SwerveConstants.kWheelRadius / 1));//0.39
}

  public double getCountsPerRev(){
    return m_backRight.getEncoderCountsPerRev();
  }

public double getEncoderCounts() {
    // Implement code to get encoder counts (sum of counts from all modules)
    // Return the total encoder counts
    return m_backRight.getDriveEncoderValues();
}

public void resetEncoders() {
  // Reset the encoders for all swerve modules
  m_frontRight.resetDriveEncoder();
  m_frontLeft.resetDriveEncoder();
  m_backLeft.resetDriveEncoder();
  m_backRight.resetDriveEncoder();
}

}

