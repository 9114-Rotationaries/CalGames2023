// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  //intake pointing away from you
  private final CANSparkMax m_rightMotor;
  private final CANSparkMax m_leftMotor;
  private final RelativeEncoder encoder1;
  // private final PIDController armPID = new PIDController(ArmConstants.p, ArmConstants.i, ArmConstants.d);

  /** Creates a new Arm. */
  public Arm(int rightArmChannel, int leftArmChannel) {
    m_rightMotor = new CANSparkMax(rightArmChannel, MotorType.kBrushless);
    m_leftMotor = new CANSparkMax(leftArmChannel, MotorType.kBrushless);
    encoder1 = m_rightMotor.getEncoder();
  }

  public void pivot(double pivotSpeed){
    //pivot speed based on right motor
    m_rightMotor.set(pivotSpeed);
    m_leftMotor.set(-pivotSpeed);
  }

  public void stop(){
    m_rightMotor.set(0);
    m_leftMotor.set(0);
  }

  public double getAngle(){
    double angle = encoder1.getPosition() / ArmConstants.armRatio;
    return angle;
  }

  public double getSpeed(){
    double speed = m_rightMotor.get();
    return speed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Angle", getAngle());
    SmartDashboard.putNumber("Arm Speed (positive = down, negative = up)", getSpeed());
  }
}