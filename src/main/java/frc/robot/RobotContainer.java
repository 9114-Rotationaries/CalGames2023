// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drivetrain.JoystickDrive;
import frc.robot.commands.Drivetrain.MoveToTag;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Arm.LowerArm;
import frc.robot.commands.Arm.RaiseArm;
import frc.robot.commands.Intake.Intake.IntakeCube;
import frc.robot.commands.Intake.Launch.LaunchCube;
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

  private final Drivetrain drivetrain = new Drivetrain();
  private final Vision vision = new Vision();

  private final Arm arm = new Arm(ArmConstants.rightArmChannel, ArmConstants.leftArmChannel);
  private final Intake intake = new Intake(IntakeConstants.cubeIntakeChannel, IntakeConstants.coneIntakeChannel);



  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    configureDrivetrainBindings();
    drivetrain.setDefaultCommand(new JoystickDrive(m_operatorController, drivetrain, true));
  }

  private void configureDrivetrainBindings() {
    m_operatorController.b().whileTrue(new MoveToTag(vision, drivetrain));
    configureIntakeBindings();
    configureArmBindings();
  }

  private void configureIntakeBindings() {
    m_operatorController.x().whileTrue(new IntakeCube(intake));
    m_operatorController.y().whileTrue(new OuttakeCube(intake));
    m_operatorController.leftBumper().whileTrue(new LaunchCube(intake));
    // m_operatorController.a().whileTrue(new IntakeCone(intake));
    // m_operatorController.b().whileTrue(new OuttakeCone(intake));
    // m_operatorController.rightBumper().whileTrue(new LaunchCone(intake));
  }

  private void configureArmBindings() {
    m_operatorController.rightTrigger().whileTrue(new RaiseArm(arm));
    m_operatorController.leftTrigger().whileTrue(new LowerArm(arm));
>>>>>>> Intake
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
