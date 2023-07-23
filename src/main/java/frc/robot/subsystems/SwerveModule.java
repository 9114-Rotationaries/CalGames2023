// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVPhysicsSim;
//import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
//import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
//import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.commands.JoystickDrive;

public class SwerveModule extends SubsystemBase {
  
  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;

  private FlywheelSim driveSim;
  private FlywheelSim turnSim;

  private final Encoder driveEncoder;
  private final Encoder turnEncoder;

  private final PIDController drivePIDController = new PIDController(ModuleConstants.PIDp, ModuleConstants.PIDi, ModuleConstants.PIDd);
  private final PIDController turnPIDController = new PIDController(ModuleConstants.ProfiledPIDp, ModuleConstants.ProfiledPIDi, ModuleConstants.ProfiledPIDd);

  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(ModuleConstants.DriveKs, ModuleConstants.DriveKv);
  private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(ModuleConstants.TurnKs, ModuleConstants.TurnKv);

  //private final JoystickDrive drive = new JoystickDrive(m_controller, null, false)

  private EncoderSim driveEncoderSim;
  private EncoderSim turnEncoderSim;

  public double turnOutput;
  public double driveOutput;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorID, int turnMotorID, int drivingEncoderChannelA, int drivingEncoderChannelB,
  int turningEncoderChannelA, int turningEncoderChannelB) {
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    //turnMotor 
    
    driveSim = new FlywheelSim(DCMotor.getNeo550(1), 1, 1);
    turnSim = new FlywheelSim(DCMotor.getNeo550(1), 1, 1);

    driveEncoder = new Encoder(drivingEncoderChannelA, drivingEncoderChannelB);
    turnEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);
    
    driveEncoderSim = new EncoderSim(driveEncoder);
    turnEncoderSim = new EncoderSim(turnEncoder);
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(driveEncoder.getRate(), new Rotation2d(turnEncoder.getDistance()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getDistance(), new Rotation2d(turnEncoder.getDistance()));
  }

  


  public void setDesiredState(SwerveModuleState desiredState){
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turnEncoder.getDistance()));
    
    driveOutput = drivePIDController.calculate(driveEncoder.getRate(), state.speedMetersPerSecond);

    double driveFeedForwardOutput = driveFeedforward.calculate(state.speedMetersPerSecond);

    turnOutput = turnPIDController.calculate(turnEncoder.getRate(), state.angle.getRadians());

    double turnFeedForwardOutput = turnFeedforward.calculate(turnPIDController.getSetpoint());

    driveMotor.set(driveOutput+driveFeedForwardOutput);
    turnMotor.set(turnOutput+turnFeedForwardOutput);
  }

  public double getDriveCurrent() {
    return driveSim.getCurrentDrawAmps();
  }

  public double getTurnCurrent() {
    return turnSim.getCurrentDrawAmps();
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
    REVPhysicsSim.getInstance().addSparkMax(driveMotor, DCMotor.getNeo550(1));
    REVPhysicsSim.getInstance().addSparkMax(turnMotor, DCMotor.getNeo550(1));
  }

  public void simulationPeriodic() {
    simulationInit(); //set up roborio / enable simulator
    REVPhysicsSim.getInstance().run();

    setDesiredState(getState()); //set next setpoint + optimize (feedforward)

    driveSim.setInputVoltage(driveMotor.get()*RobotController.getBatteryVoltage());
    turnSim.setInputVoltage(turnMotor.get()*RobotController.getBatteryVoltage());

    double drawCurrent = getDriveCurrent()+getTurnCurrent();
    double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(drawCurrent);
    RoboRioSim.setVInVoltage(loadedVoltage);

    System.out.println("Working");

    //REVPhysicsSim.getInstance().run();
    //driveSim.setInputVoltage(getDriveSpeed());
    // This method will be called once per scheduler run
    
    //turnMotor.set(turnOutput / ModuleConstants.kModuleMaxAngularVelocity * RobotController.getBatteryVoltage());

    //TalonFXControlMode

    //driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, frameTime);
  }
}