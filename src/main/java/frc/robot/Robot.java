// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Balance;
import frc.robot.subsystems.Vision;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_balanceCode;
  //private Command joystickDrive;

  private RobotContainer m_robotContainer;
  private Vision limelight;

  //private final Drivetrain m_swerve = new Drivetrain();
  WaitCommand x = new WaitCommand(5);
  

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    limelight = new Vision();
    m_robotContainer.setUpAutos();
    PathPlannerServer.startServer(9114);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_balanceCode = m_robotContainer.balanceCode();

    if (m_autonomousCommand != null) {
      //Commands.sequence(m_autonomousCommand, m_balanceCode);
      m_autonomousCommand.schedule();
    }
    //m_robotContainer.balance.execute();
    //CommandScheduler.getInstance().schedule(m_robotContainer.balanceCode());
  
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //CommandScheduler.getInstance().schedule(x);
    //
  }

  @Override
  public void teleopPeriodic() {
    //CommandScheduler.getInstance().schedule(x);
    //CommandScheduler.getInstance().run();
    //joystickDrive.schedule();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
