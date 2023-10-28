// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SimConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DrivetrainSim extends SubsystemBase {

  private final SwerveModuleSim m_frontLeft = new SwerveModuleSim();
  private final SwerveModuleSim m_frontRight = new SwerveModuleSim();
  private final SwerveModuleSim m_backLeft = new SwerveModuleSim();
  private final SwerveModuleSim m_backRight = new SwerveModuleSim();

  private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

  public Field2d field = new Field2d();

  private final SwerveDriveKinematics m_simKinematics =
      new SwerveDriveKinematics(
          SimConstants.m_frontLeftLocation, SimConstants.m_frontRightLocation, SimConstants.m_backLeftLocation, SimConstants.m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_simKinematics,
          new Rotation2d(0),
          getModulePositions());

  /** Creates a new DrivetrainSim. */
  public DrivetrainSim() {
    ahrs.reset();
    SmartDashboard.putData("Field", field);
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    modulePositions[0] = m_frontLeft.getModulePosition();
    modulePositions[1] = m_frontRight.getModulePosition();
    modulePositions[2] = m_backLeft.getModulePosition();
    modulePositions[3] = m_backRight.getModulePosition();
    return modulePositions;
  }
  
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_simKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, ahrs.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SimConstants.kMaxSpeed);
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
    return m_odometry.getPoseMeters();
  }
  
  @Override
  public void periodic(){
    updateOdometry();
    field.setRobotPose(m_odometry.getPoseMeters());
    //SmartDashboard.putData(field);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SimConstants.kMaxAngularSpeed);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    //SmartDashboard.putNumber("encoderRotConverte", m_backRight.getDrivePosition());
    //SmartDashboard.putNumber("encoderRotRaw", m_backRight.getDriveEncoder());
    //SmartDashboard.putString("ModulePosition", m_backRight.getModulePosition().toString());
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


  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getYaw());
  }

 


}
