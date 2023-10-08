// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drivetrain.JoystickDrive;
import frc.robot.commands.Drivetrain.MoveToTag;
import frc.robot.commands.Drivetrain.SlowDriveCommunity;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;                            

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Arm.LowerArm;
import frc.robot.commands.Arm.RaiseArm;
import frc.robot.commands.Autos.Balance;
import frc.robot.commands.Autos.Macros;
import frc.robot.commands.Autos.Routines;
import frc.robot.commands.Intake.Intake.IntakeCube;
import frc.robot.commands.Intake.Outtake.OuttakeCube;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final static Drivetrain drivetrain = new Drivetrain();
  private final Vision vision = new Vision();
  private final static Arm arm = new Arm(ArmConstants.rightArmChannel, ArmConstants.leftArmChannel);
  private final static Intake intake = new Intake(IntakeConstants.cubeIntakeChannel, IntakeConstants.coneIntakeChannel);

  private final static Macros macros = new Macros(arm, intake);
  private final static Balance balance = new Balance(drivetrain);
  private final static Routines routines = new Routines(drivetrain, intake, arm, macros);


  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final CommandXboxController m_controller = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureDrivetrainBindings();
    configureArmBindings();
    configureIntakeBindings();
    drivetrain.setDefaultCommand(new JoystickDrive(m_controller, drivetrain, true));
  }

  private void configureDrivetrainBindings() {
    m_controller.b().whileTrue(new MoveToTag(vision, drivetrain));
    configureIntakeBindings();
    configureArmBindings();
    m_controller.button(9).whileTrue(new SlowDriveCommunity(m_controller, drivetrain, true));
  }

  private void configureIntakeBindings() {
    m_operatorController.leftBumper().whileTrue(new IntakeCube(intake));
    m_controller.rightTrigger().whileTrue(new IntakeCube(intake));
    m_controller.leftTrigger().whileTrue(new OuttakeCube(intake));
    m_operatorController.rightBumper().whileTrue(new OuttakeCube(intake));
  }

  private void configureArmBindings() {
    m_operatorController.rightTrigger().whileTrue(new RaiseArm(arm));
    m_operatorController.leftTrigger().whileTrue(new LowerArm(arm));
    m_controller.leftBumper().whileTrue(new LowerArm(arm));
    m_controller.rightBumper().whileTrue(new RaiseArm(arm));
  }

  public Command getAutonomousCommand() {
    return routines.MidDock(drivetrain);
    }

  public Command balanceCode(){
    return balance;
  }
}
