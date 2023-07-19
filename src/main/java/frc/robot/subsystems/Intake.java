// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_indexMotor;
  private final CANSparkMax m_intake1;
  private final CANSparkMax m_intake2;


  /** Creates a new Intake. */
  public Intake(int indexMotorChannel, int intake1Channel, int intake2Channel) {
    m_indexMotor = new CANSparkMax(indexMotorChannel, MotorType.kBrushless);
    m_intake1 = new CANSparkMax(intake1Channel, MotorType.kBrushless);
    m_intake2 = new CANSparkMax(intake2Channel, MotorType.kBrushless);
  }

  public void intake(double indexSpeed, double intakeSpeed){
    m_indexMotor.set(indexSpeed);
    m_intake1.set(intakeSpeed);
    m_intake2.set(-intakeSpeed);
  }

  public void outtake(double indexSpeed, double intakeSpeed){
    m_indexMotor.set(indexSpeed);
    m_intake1.set(intakeSpeed);
    m_intake2.set(-intakeSpeed);
  }

  public void startFlywheels(double speed){
    m_intake1.set(speed);
    m_intake2.set(-speed);
  }

  public void startIndexer(double speed){
    m_indexMotor.set(speed);
  }

  public void stop(){
    m_indexMotor.set(0);
    m_intake1.set(0);
    m_intake2.set(0);
  }

  public double getFlywheelSpeed(){
    return m_intake1.get();
  }

  public double getIndexSpeed(){
    return m_indexMotor.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Flywheel Speed", getFlywheelSpeed());
    SmartDashboard.putNumber("Intake Indexer Speed", getIndexSpeed());
    // This method will be called once per scheduler run
  }
}
