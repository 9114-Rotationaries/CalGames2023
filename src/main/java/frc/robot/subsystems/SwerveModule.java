// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
//import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
//import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
//import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.CANCoder;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.JoystickDrive;

public class SwerveModule extends SubsystemBase {
  
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private FlywheelSim driveSim;
  private FlywheelSim turnSim;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;
  private final CANCoder m_angleEncoder;

  private final PIDController m_drivePIDController = new PIDController(SwerveConstants.PIDp, SwerveConstants.PIDi, SwerveConstants.PIDd);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          SwerveConstants.ProfiledPIDp,
          SwerveConstants.ProfiledPIDi,
          SwerveConstants.ProfiledPIDd,
          new TrapezoidProfile.Constraints(
              SwerveConstants.kModuleMaxAngularVelocity, SwerveConstants.kModuleMaxAngularAcceleration));

  //private Encoder fakeDriveEncoder = new Encoder(1, 2, 0, true);
  //private Encoder fakeTurnEncoder = new Encoder(3, 4, 5, true);

  //private final JoystickDrive drive = new JoystickDrive(m_controller, null, false)

  //private EncoderSim driveEncoderSim = new EncoderSim(fakeDriveEncoder);
  //private EncoderSim turnEncoderSim = new EncoderSim(fakeTurnEncoder);

  public double turnOutput;
  public double driveOutput;

  private double m_simDriveEncoderPosition;
  private double m_simDriveEncoderVelocity;
  private double m_simAngleDifference;
  private double m_simTurnAngleIncrement;

  //private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  /** Creates a new SwerveModule. */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double turningEncoderOffsetDegrees) {
    
    
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    //m_driveMotor.setInverted(true);
    m_turningMotor.setInverted(true);

    m_driveEncoder = m_driveMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    m_turningEncoder = m_turningMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    // Distance per pulse
    m_driveEncoder.setPositionConversionFactor(2 * Math.PI * SwerveConstants.kWheelRadius / 6.75);
    m_driveEncoder.setVelocityConversionFactor(2 * Math.PI * SwerveConstants.kWheelRadius / 6.75 / 60); // 60 seconds per minute

    driveSim = new FlywheelSim(DCMotor.getNeo550(1), 1, 1);
    turnSim = new FlywheelSim(DCMotor.getNeo550(1), 1, 1);

    // Radians per pulse
     //m_turningEncoder.setDistancePerPulse(2 * Math.PI / SwerveConstants.kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI/2, Math.PI/2);

    //m_turningEncoder.configMagnetOffset(-turningEncoderOffsetDegrees);
  }

  public SwerveModuleState getState() {
    double m_moduleAngleRadians = m_angleEncoder.getAbsolutePosition() * 2 * Math.PI / 360;

    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(m_moduleAngleRadians));
  }

  public SwerveModulePosition getPosition() {
    double m_moduleAngleRadians = m_angleEncoder.getAbsolutePosition() * 2 * Math.PI / 360;

    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(m_moduleAngleRadians));
  }

  public double getDriveMeters() {
    if(RobotBase.isReal())
      return m_driveEncoder.getPosition();
    else
        return m_simDriveEncoderPosition;
  }
  public double getDriveMetersPerSecond() {
    if(RobotBase.isReal())
      return m_driveEncoder.getVelocity();
    else
      return m_simDriveEncoderVelocity;
  }

  private void simUpdateDrivePosition(SwerveModuleState state) {
    m_simDriveEncoderVelocity = state.speedMetersPerSecond;
    double distancePer20Ms = m_simDriveEncoderVelocity / 50.0;

    m_simDriveEncoderPosition += distancePer20Ms;
  }
  private void simTurnPosition(double angle) {
    if (angle != m_currentAngle && m_simTurnAngleIncrement == 0) {
      m_simAngleDifference = angle - m_currentAngle;
      m_simTurnAngleIncrement = m_simAngleDifference / 20.0;// 10*20ms = .2 sec move time
    }

    if (m_simTurnAngleIncrement != 0) {
      m_currentAngle += m_simTurnAngleIncrement;

      if ((Math.abs(angle - m_currentAngle)) < .1) {
        m_currentAngle = angle;
        m_simTurnAngleIncrement = 0;
      }
    }

    private void simUpdateDrivePosition(SwerveModuleState state) {
      m_simDriveEncoderVelocity = state.speedMetersPerSecond;
      double distancePer20Ms = m_simDriveEncoderVelocity / 50.0;
  
      m_simDriveEncoderPosition += distancePer20Ms;
    }
    private void simTurnPosition(double angle) {
      if (angle != m_currentAngle && m_simTurnAngleIncrement == 0) {
        m_simAngleDifference = angle - m_currentAngle;
        m_simTurnAngleIncrement = m_simAngleDifference / 20.0;// 10*20ms = .2 sec move time
      }
  
      if (m_simTurnAngleIncrement != 0) {
        m_currentAngle += m_simTurnAngleIncrement;
  
        if ((Math.abs(angle - m_currentAngle)) < .1) {
          m_currentAngle = angle;
          m_simTurnAngleIncrement = 0;
        }
      }
    }
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    double m_moduleAngleRadians = m_angleEncoder.getAbsolutePosition() * 2 * Math.PI / 360;


    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_moduleAngleRadians));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);
    
    
    // final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_moduleAngleRadians, state.angle.getRadians());

    // final double turnFeedforward =
    //     m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_turningMotor.set(turnOutput);
    //System.out.println(turnOutput);
    m_driveMotor.set(driveOutput);
  }

  public double getDriveCurrent() {
    return driveSim.getCurrentDrawAmps();
  }

  public double getTurnCurrent() {
    return turnSim.getCurrentDrawAmps();
  }

  public double getHeadingDegrees() {
    if(RobotBase.isReal())
      return m_turningEncoder.getPosition();
    else
      return m_currentAngle;
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  

  /*public CANSparkMax getDriveMotor(){
    return driveMotor;
  }

  public CANSparkMax getTurnMotor(){
    return turnMotor;
  }

  public double getDriveSpeed(){
    return driveMotor.get();
  }

  public double getTurnSpeed(){
    return driveMotor.get();
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void simulationInit() {
    REVPhysicsSim.getInstance().addSparkMax(m_driveMotor, DCMotor.getNeo550(1));
    REVPhysicsSim.getInstance().addSparkMax(m_turningMotor, DCMotor.getNeo550(1));
  }

  public void simulationPeriodic() {
    simulationInit(); //set up roborio / enable simulator
    REVPhysicsSim.getInstance().run();

    driveSim.setInputVoltage(m_driveMotor.get()*RobotController.getBatteryVoltage());
    turnSim.setInputVoltage(m_turningMotor.get()*RobotController.getBatteryVoltage());

    double drawCurrent = getDriveCurrent()+getTurnCurrent();
    double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(drawCurrent);
    RoboRioSim.setVInVoltage(loadedVoltage);

    //setDesiredState(getState()); //set next setpoint w/ position
    //System.out.println("Working");

    //REVPhysicsSim.getInstance().run();
    //driveSim.setInputVoltage(getDriveSpeed());
    // This method will be called once per scheduler run
    
    //turnMotor.set(turnOutput / ModuleConstants.kModuleMaxAngularVelocity * RobotController.getBatteryVoltage());

    //TalonFXControlMode

    //driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, frameTime);
  }
}