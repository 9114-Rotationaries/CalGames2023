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


  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getYaw());
  }

  public double getDistancePerPulse() {
    // Return the conversion factor for distance per pulse
    return 4096 *6.75 /((2 * Math.PI * SwerveConstants.kWheelRadius / 1)*(2 * Math.PI * SwerveConstants.kWheelRadius / 1));//0.39
}

}

