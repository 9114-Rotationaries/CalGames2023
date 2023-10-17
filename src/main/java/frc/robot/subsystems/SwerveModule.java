package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {
  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor; 

  //private final RelativeEncoder m_driveEncoder;
  private final CANCoder m_turningEncoder;

  private final PIDController m_drivePIDController = new PIDController(SwerveConstants.PIDp, SwerveConstants.PIDi, SwerveConstants.PIDd);

  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          SwerveConstants.ProfiledPIDp,
          SwerveConstants.ProfiledPIDi,
          SwerveConstants.ProfiledPIDd,
          new TrapezoidProfile.Constraints(
              SwerveConstants.kModuleMaxAngularVelocity, SwerveConstants.kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  //private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.DriveKs, SwerveConstants.DriveKv);
  //private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(SwerveConstants.TurnKs, SwerveConstants.TurnKv);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, and turning encoder.
   *
   * @param driveMotorChannel CAN ID for the drive motor
   * @param turningMotorChannel CAN ID for the turning motor
   * @param turningEncoderChannel CAN ID for the turning encoder
   * @param turningEncoderOffsetDegrees degrees of offset for the absolute encoder
   */

  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double turningEncoderOffsetDegrees) {
    
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);
    m_turningMotor.setInverted(true);
    
    //m_driveMotor.sensor
    //m_driveEncoder = m_driveMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    m_turningEncoder = new CANCoder(turningEncoderChannel);

    
    //m_driveEncoder.setPositionConversionFactor();//42/(6.75 * Math.PI * SwerveConstants.kWheelRadius * 2) and also that but change 42 to 1//((Math.PI * SwerveConstants.kWheelRadius * 2)/6.75)/5
    //m_driveEncoder.setVelocityConversionFactor();//((/ 60); // 60 seconds per minute

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    //m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    //m_turningEncoder.configMagnetOffset(-turningEncoderOffsetDegrees);
  }

  public SwerveModuleState getState() {

    double m_moduleAngleRadians = m_turningEncoder.getAbsolutePosition() * 2 * Math.PI / 360;
    //double m_moduleAngleRadians = m_turningMotor.getSelectedSensorPosition();

    return new SwerveModuleState(
      getDriveVelocity(), new Rotation2d(m_moduleAngleRadians));
  }

  public SwerveModulePosition getPosition() {

    double m_moduleAngleRadians = m_turningEncoder.getAbsolutePosition() * 2 * Math.PI / 360;

    return new SwerveModulePosition(
      getDrivePosition(), new Rotation2d(m_moduleAngleRadians));
  }

  public double getStateRotation(SwerveModuleState desiredState){
    
    return desiredState.angle.getRadians();
  }
  
  public void setDesiredState(SwerveModuleState desiredState) {
    double m_moduleAngleRadians = m_turningEncoder.getAbsolutePosition() * 2 * Math.PI / 360;


    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_moduleAngleRadians));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond);
    
    
    // final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    SmartDashboard.putNumber("thign", getDriveVelocity());
    SmartDashboard.putNumber("2thign", state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_moduleAngleRadians, state.angle.getRadians());

    // final double turnFeedforward =
    //     m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_turningMotor.set(ControlMode.Velocity, driveOutput);;
    m_driveMotor.set(ControlMode.Velocity, turnOutput);
  } 


  public double getDrivePosition(){
    return m_driveMotor.getSelectedSensorPosition() * 2*Math.PI*0.0508/6.75;
  }

  public double getDriveVelocity() {
    return m_driveMotor.getSelectedSensorVelocity() * 2*Math.PI*0.0508/6.75/60;
  }

  public double getTurningPosition(){
    double m_moduleAngleRadians = m_turningEncoder.getAbsolutePosition() * 2 * Math.PI / 360;
    return m_moduleAngleRadians;
  }

  // public double getDriveEncoder(){
  //   return m_driveEncoder.getPosition();
  // }
  
  public SwerveModulePosition getModulePosition() {
    SwerveModulePosition modulePosition = new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRadians(getTurningPosition()));
    return modulePosition;
  } 
}
