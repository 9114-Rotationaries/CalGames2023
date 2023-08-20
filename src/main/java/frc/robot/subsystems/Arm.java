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

  private final CANSparkMax m_motor1;
  private final CANSparkMax m_motor2;
  private final RelativeEncoder encoder1;
  // private final PIDController armPID = new PIDController(ArmConstants.p, ArmConstants.i, ArmConstants.d);

  /** Creates a new Arm. */
  public Arm(int armMotor1Channel, int armMotor2Channel) {
    m_motor1 = new CANSparkMax(armMotor1Channel, MotorType.kBrushless);
    m_motor2 = new CANSparkMax(armMotor2Channel, MotorType.kBrushless);
    encoder1 = m_motor1.getEncoder();
  }

  public void pivot(double pivotSpeed){
    m_motor1.set(pivotSpeed);
    m_motor2.set(-pivotSpeed);
  }

  public void stop(){
    m_motor1.set(0);
    m_motor2.set(0);
  }

  public double getAngle(){
    double angle = encoder1.getPosition() / ArmConstants.armRatio;
    return angle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Angle", getAngle());
  }
}