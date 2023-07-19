// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Arm.LowerArm;
import frc.robot.commands.Arm.RaiseArm;
import frc.robot.commands.Intake.IntakeObject;
import frc.robot.commands.Intake.LaunchObject;
import frc.robot.commands.Intake.OuttakeObject;
import frc.robot.commands.Intake.RevFlywheels;
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
  private final Arm arm = new Arm(ArmConstants.armMotorChannel);
  private final Intake intake = new Intake(IntakeConstants.indexMotorChannel, IntakeConstants.intake1Channel, IntakeConstants.intake2Channel);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureIntakeBindings();
    configureArmBindings();
  }

  private void configureIntakeBindings() {
    m_driverController.a().whileTrue(new IntakeObject(intake));
    m_driverController.y().whileTrue(new OuttakeObject(intake));
    m_driverController.rightBumper().whileTrue(new RevFlywheels(intake));
    m_driverController.leftBumper().whileTrue(new LaunchObject(intake));
  }

  private void configureArmBindings() {
    m_driverController.rightTrigger().whileTrue(new RaiseArm(arm));
    m_driverController.leftTrigger().whileTrue(new LowerArm(arm));
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
