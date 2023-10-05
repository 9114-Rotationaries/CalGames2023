// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_cubeIntake;
  private final Timer timer = new Timer();

  // private final CANSparkMax m_coneIntake;

  /** Creates a new Intake. */
  public Intake(int cubeIntakeChannel, int coneIntakeChannel) {
    m_cubeIntake = new CANSparkMax(cubeIntakeChannel, MotorType.kBrushless);
    // m_coneIntake = new CANSparkMax(coneIntakeChannel, MotorType.kBrushless);
  }

  public void cubeIntake(double cubeIntakeSpeed){
    m_cubeIntake.set(cubeIntakeSpeed);
  }

  public void cubeOuttake(double cubeOuttakeSpeed){
    m_cubeIntake.set(cubeOuttakeSpeed);
  }

  public void cubeLaunch(double cubeLaunchSpeed){
    m_cubeIntake.set(cubeLaunchSpeed);
  }

  public double getCubeIntakeSpeed(){
    return m_cubeIntake.get();
  }

  public void cOut(){ 
    timer.reset();
    timer.start();
    while (timer.get() < 2){
      m_cubeIntake.set(.8);    
    }
    m_cubeIntake.set(0); //test auton code
  }

  public void cInt(){
    timer.reset();
    timer.start();
    while (timer.get() < 2){
      m_cubeIntake.set(-.8);    
    }
    m_cubeIntake.set(0); 
  }

  // public void coneIntake(double coneIntakeSpeed){
  //   m_cubeIntake.set(coneIntakeSpeed);
  // }

  // public void coneOuttake(double coneOuttakeSpeed){
  //   m_cubeIntake.set(coneOuttakeSpeed);
  // }

  // public void coneLaunch(double coneLaunchSpeed){
  //   m_cubeIntake.set(coneLaunchSpeed);
  // }

  // public double getConeIntakeSpeed(){
  //   return m_coneIntake.get();
  // }

  public void stop(){
    m_cubeIntake.set(0);
    // m_coneIntake.set(0);
  }


  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Cube Intake Speed", getCubeIntakeSpeed());
    // SmartDashboard.putNumber("Cone Intake Speed", getConeIntakeSpeed());
    // This method will be called once per scheduler run
  }
}
