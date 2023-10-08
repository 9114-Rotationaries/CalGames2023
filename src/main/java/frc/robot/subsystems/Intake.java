// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_cubeIntake;

  public Intake(int cubeIntakeChannel, int coneIntakeChannel) {
    m_cubeIntake = new CANSparkMax(cubeIntakeChannel, MotorType.kBrushless);
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

  public void stop(){
    m_cubeIntake.set(0);
  }

  @Override
  public void periodic() {
  }
}
