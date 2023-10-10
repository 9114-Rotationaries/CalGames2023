package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_cubeIntake;
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
    m_cubeIntake.set(.4);  
    Commands.runOnce(() -> new WaitCommand(.5)); 
    m_cubeIntake.set(0); //test auton code
  }

  public void cInt(){
    m_cubeIntake.set(-.4);
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
  public void periodic() {}
}
