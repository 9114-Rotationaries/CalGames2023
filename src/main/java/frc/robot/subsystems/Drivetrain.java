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
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
          new Rotation2d(0),
          getModulePositions());

  public Drivetrain() {
    ahrs.reset();
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    modulePositions[0] = m_frontLeft.getModulePosition();
    modulePositions[1] = m_frontRight.getModulePosition();
    modulePositions[2] = m_backLeft.getModulePosition();
    modulePositions[3] = m_backRight.getModulePosition();
    return modulePositions;
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


    
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(getRotation2d(), getModulePositions());
  } 

  public Pose2d getPose(){
    //updateOdometry();
    return m_odometry.getPoseMeters();
  }
  
  @Override
  public void periodic(){
    updateOdometry();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxAngularSpeed);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    SmartDashboard.putNumber("encoderRotConverte", m_backRight.getDrivePosition());
    SmartDashboard.putNumber("encoderRotRaw", m_backRight.getDriveEncoder());
    SmartDashboard.putString("ModulePosition", m_backRight.getModulePosition().toString());
  }

  public SwerveDriveKinematics getKinematics(){
    return m_kinematics;
  }

  public float getPitch(){
    return ahrs.getPitch(); 
  }

  public float getYaw(){
    return ahrs.getYaw(); 
  }

  public float getRoll(){
    return ahrs.getRoll(); 
  }

  public double getHeading() {
    double angle = ahrs.getYaw() % 360;
    if (angle > 180) {
      angle -= 360;
    } else if (angle <= -180) {
        angle += 360;
    }
    return angle;
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public double getDistancePerPulse() {
    // Return the conversion factor for distance per pulse
    return 4096 *6.75 /((2 * Math.PI * SwerveConstants.kWheelRadius / 1)*(2 * Math.PI * SwerveConstants.kWheelRadius / 1));//0.39
}

  // public double getCountsPerRev(){
  //   return m_backRight.getEncoderCountsPerRev();
  // }

// public double getEncoderCounts() {
//     // Implement code to get encoder counts (sum of counts from all modules)
//     // Return the total encoder counts
//     return m_backRight.getDriveEncoderValues();
// }

// public void resetEncoders() {
//   // Reset the encoders for all swerve modules
//   m_frontRight.resetDriveEncoder();
//   m_frontLeft.resetDriveEncoder();
//   m_backLeft.resetDriveEncoder();
//   m_backRight.resetDriveEncoder();
// }

}

