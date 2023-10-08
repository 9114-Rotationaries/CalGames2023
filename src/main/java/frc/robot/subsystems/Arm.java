// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  //intake pointing away from you
  private final CANSparkMax m_rightMotor;
  private final CANSparkMax m_leftMotor;
  private final RelativeEncoder encoderRight;
  private final RelativeEncoder encoderLeft;
  private final PIDController armPIDController;

  /** Creates a new Arm. */
  public Arm(int rightArmChannel, int leftArmChannel) {
    m_rightMotor = new CANSparkMax(rightArmChannel, MotorType.kBrushless);
    m_leftMotor = new CANSparkMax(leftArmChannel, MotorType.kBrushless);
    encoderRight = m_rightMotor.getEncoder();
    encoderLeft = m_leftMotor.getEncoder();
    armPIDController = new PIDController(ArmConstants.armPIDp, ArmConstants.armPIDi, ArmConstants.armPIDd);
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

  @Override
  public void periodic() {
  }
}