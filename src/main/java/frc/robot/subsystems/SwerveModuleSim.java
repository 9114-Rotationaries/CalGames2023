// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.SimConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveModuleIO.ModuleIOInputs;


public class SwerveModuleSim implements SwerveModuleIO {
  private FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
  private FlywheelSim turnSim = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);



  private double turnRelativePositionRad = 0.0;
  private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private final PIDController m_drivePIDController = new PIDController(SimConstants.DrivePIDp, SimConstants.DrivePIDi, SimConstants.DrivePIDd);
  private final PIDController m_turningPIDController = new PIDController(SimConstants.TurnPIDp, SimConstants.TurnPIDi, SimConstants.TurnPIDd);

  public SwerveModuleSim() {
    System.out.println("[Init] Creating ModuleIOSim");
  }

  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.update(SimConstants.loopPeriodSecs);
    turnSim.update(SimConstants.loopPeriodSecs);

    double angleDiffRad = turnSim.getAngularVelocityRadPerSec() * SimConstants.loopPeriodSecs;
    turnRelativePositionRad += angleDiffRad;
    turnAbsolutePositionRad += angleDiffRad;
    while (turnAbsolutePositionRad < 0) {
      turnAbsolutePositionRad += 2.0 * Math.PI;
    }
    while (turnAbsolutePositionRad > 2.0 * Math.PI) {
      turnAbsolutePositionRad -= 2.0 * Math.PI;
    }

    inputs.drivePositionRad =
        inputs.drivePositionRad
            + (driveSim.getAngularVelocityRadPerSec() * SimConstants.loopPeriodSecs);
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(driveSim.getCurrentDrawAmps())};
    inputs.driveTempCelcius = new double[] {};

    inputs.turnAbsolutePositionRad = turnAbsolutePositionRad;
    inputs.turnPositionRad = turnRelativePositionRad;
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = new double[] {Math.abs(turnSim.getCurrentDrawAmps())};
    inputs.turnTempCelcius = new double[] {};
  }

  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    double m_moduleAngleRadians = turnSim.getAngularVelocityRadPerSec() * 2 * Math.PI / 360;


    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_moduleAngleRadians));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(driveSim.getAngularVelocityRadPerSec(), state.speedMetersPerSecond);
    
    
    // final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    //SmartDashboard.putNumber("thign", m_driveEncoder.getVelocity());
    //SmartDashboard.putNumber("2thign", state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_moduleAngleRadians, state.angle.getRadians());

    // final double turnFeedforward =
    //     m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    turnSim.setInputVoltage(turnOutput*12);
    driveSim.setInputVoltage(driveOutput*12);
  } 

  //rpm --> rps --> rotations --> distance
  public double getDrivePosition(){
    return driveSim.getAngularVelocityRPM()/60*SimConstants.loopPeriodSecs*2*Math.PI*SwerveConstants.kWheelRadius;
  }

  public double getTurningPosition(){
    double m_moduleAngleRadians = turnSim.getAngularVelocityRPM() * 2 * Math.PI / 360;
    return m_moduleAngleRadians;
  }

  
public SwerveModulePosition getModulePosition() {
  SwerveModulePosition modulePosition = new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRadians(getTurningPosition()));
  return modulePosition;
} 
}