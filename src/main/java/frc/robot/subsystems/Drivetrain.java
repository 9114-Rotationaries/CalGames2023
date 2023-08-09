// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
//import com.revrobotics.REVPhysicsSim;

//import edu.wpi.first.wpilibj.simulation.FlywheelSim;
//import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
//import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
//import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
//import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.commands.JoystickDrive;

public class Drivetrain extends SubsystemBase {

  private final Field2d m_field = new Field2d();

  //public DifferentialDrivetrainSim drivetrainSim;

  public final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381); //meters
  public final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
  public final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
  public final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule[] swerveModules = {
    new SwerveModule(
    DriveConstants.FLDMChannel, DriveConstants.FLTMChannel, 
    DriveConstants.FLTEChannel, DriveConstants.FLTEOffsetDegrees),
    new SwerveModule(
    DriveConstants.FRDMChannel, DriveConstants.FRTMChannel, 
    DriveConstants.FRTEChannel, DriveConstants.FRTEOffsetDegrees),
    new SwerveModule(
    DriveConstants.BLDMChannel, DriveConstants.BLTMChannel, 
    DriveConstants.BLTEChannel, DriveConstants.BLTEOffsetDegrees),
    new SwerveModule(
    DriveConstants.BRDMChannel, DriveConstants.BRTMChannel, 
    DriveConstants.BRTEChannel, DriveConstants.BRTEOffsetDegrees)};

  //private final AHRS ahrs = new AHRS(SPI.Port.kMXP);
  private Pigeon2 m_pigeon = new Pigeon2(DriveConstants.pigeon);

  //AHRS is not analog gyro
  //private final AnalogGyroSim ahrsSim = new AnalogGyroSim(ahrs);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    m_kinematics, 
    getHeadingRotation2d(), 
    new SwerveModulePosition[] {
            swerveModules[0].getPosition(),
            swerveModules[1].getPosition(),
            swerveModules[2].getPosition(),
            swerveModules[3].getPosition()
          });


  /** Creates a new Drivetrain. */
  public Drivetrain() {
    //ahrs.reset();

    SmartDashboard.putData("Field", m_field);
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
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeadingRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeed);

    swerveModules[0].setDesiredState(swerveModuleStates[0]);
    swerveModules[1].setDesiredState(swerveModuleStates[1]);
    swerveModules[2].setDesiredState(swerveModuleStates[2]);
    swerveModules[3].setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        getHeadingRotation2d(),
        new SwerveModulePosition[] {
          swerveModules[0].getPosition(),
          swerveModules[1].getPosition(),
          swerveModules[2].getPosition(),
          swerveModules[3].getPosition()
        });
  }

  public Pose2d getPoseMeters(){
    return m_odometry.getPoseMeters();
  }

  public SwerveModule getSwerveModule(int idx) {
    return swerveModules[idx];
  }

  public double getHeadingDegrees() {
    return Math.IEEEremainder(m_pigeon.getYaw(), 360);
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  /*public void simulationInit() {
    REVPhysicsSim.getInstance().addSparkMax(null, null);
  }*/

  public void simulationPeriodic(){
    //driveSim.setInputVoltage(frontLeft.getDriveSpeed() * RobotController.getInputVoltage());
    //drivetrainSim.update(0.02);
    if (RobotBase.isSimulation()) { // If our robot is simulated
      //System.out.println("Running");
      // This class simulates our drivetrain's motion around the field.
      /*driveSim = new FlywheelSim(DriveConstants.kDrivetrainPlant, DriveConstants.kDriveGearbox, DriveConstants.kDriveGearing, 
                        DriveConstants.kTrackwidthMeters,
                        DriveConstants.kWheelDiameterMeters / 2.0,
                        VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));*/
      swerveModules[0].simulationPeriodic();
      swerveModules[1].simulationPeriodic();
      swerveModules[2].simulationPeriodic();
      swerveModules[3].simulationPeriodic();
      drive(0.5, 0.5, 0.5, true);
    }

    updateOdometry();
    //System.out.println(m_odometry.getPoseMeters());
    m_field.setRobotPose(m_odometry.getPoseMeters());

  }
}