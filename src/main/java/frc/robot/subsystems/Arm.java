// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private final CANSparkMax m_motor;
  private final RelativeEncoder encoder;
  // private final PIDController armPID = new PIDController(ArmConstants.p, ArmConstants.i, ArmConstants.d);

  /** Creates a new Arm. */
  public Arm(int motorChannel) {
    m_motor = new CANSparkMax(motorChannel, MotorType.kBrushless);
    encoder = m_motor.getEncoder();
  }

  public void pivot(double speed){
    m_motor.set(speed);
  }

  public void stop(){
    m_motor.set(0);
  }

  public double getAngle(){
    double angle = encoder.getPosition() / ArmConstants.armRatio;
    return angle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}