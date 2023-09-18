// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.commands.Auto.*;
import frc.robot.commands.Intake.Intake.IntakeCube;
import frc.robot.commands.Intake.Launch.LaunchCone;
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

  private final static Drivetrain drivetrain = new Drivetrain();
  private final Vision vision = new Vision();

  private final Arm arm = new Arm(ArmConstants.rightArmChannel, ArmConstants.leftArmChannel);
  private final Intake intake = new Intake(IntakeConstants.cubeIntakeChannel, IntakeConstants.coneIntakeChannel);
  
  private SwerveAutoBuilder builder;
  SendableChooser<List<PathPlannerTrajectory>> autoChooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private final CommandXboxController m_controller = new CommandXboxController(OperatorConstants.kDriverControllerPort);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    //drivetrain.resetPose(drivetrain.getPose());
    System.out.println(drivetrain.getModule().getPosition());

    configureDrivetrainBindings();
    drivetrain.setDefaultCommand(new JoystickDrive(m_controller, drivetrain, true));

    configureBuilder();
    configureChooser();
  }

  private void configureDrivetrainBindings() {
    m_controller.b().whileTrue(new MoveToTag(vision, drivetrain));
    configureIntakeBindings();
    configureArmBindings();
  }

  private void configureIntakeBindings() {
    m_operatorController.x().whileTrue(new IntakeCube(intake));
    m_controller.rightTrigger().whileTrue(new IntakeCube(intake));
    m_operatorController.y().whileTrue(new OuttakeCube(intake));
    m_operatorController.rightBumper().whileTrue(new LaunchCube(intake));
    // m_operatorController.a().whileTrue(new IntakeCone(intake));
    // m_operatorController.b().whileTrue(new OuttakeCone(intake));
    m_operatorController.leftBumper().whileTrue(new LaunchCone(intake));
  }

  private void configureArmBindings() {
    m_operatorController.rightTrigger().whileTrue(new RaiseArm(arm));
    m_operatorController.leftTrigger().whileTrue(new LowerArm(arm));
  }

  public void configureBuilder() {
    builder = new SwerveAutoBuilder(
      drivetrain::getPose,
      drivetrain::resetOdometry,
      drivetrain.getKinematics(),
      new PIDConstants(0.7, 0.0001, 0.0),
      new PIDConstants(0.1, 0.0001, 0),
      drivetrain::setModuleStates,
      new HashMap<String, Command>(),
      true,
      drivetrain
    );
  }

  public void configureChooser(){
    //autoChooser.setDefaultOption("Do Nothing", new WaitCommand(15));
    autoChooser.addOption("Go Forward", PathPlanner.loadPathGroup("Score1LeftBalance", new PathConstraints(4, 3)));    autoChooser.addOption("1CO1CU-B", (Command) new Co1Cu1B(builder));
    autoChooser.addOption("1CO1CU-M", PathPlanner.loadPathGroup("1CO1CU-M", new PathConstraints(4, 3)));
    autoChooser.addOption("1CO1CU-B", PathPlanner.loadPathGroup("1CO1CU-B", new PathConstraints(4, 3)));
    autoChooser.addOption("1CO1CU-T", PathPlanner.loadPathGroup("1CO1CU-T", new PathConstraints(4, 3)));
    autoChooser.addOption("2CO-B", PathPlanner.loadPathGroup("2CO-B", new PathConstraints(4, 3)));
    autoChooser.addOption("2CO-M", PathPlanner.loadPathGroup("2CO-M", new PathConstraints(4, 3)));
    autoChooser.addOption("2CO-T", PathPlanner.loadPathGroup("2CO-T", new PathConstraints(4, 3)));
    autoChooser.addOption("2CO1CU-B", PathPlanner.loadPathGroup("2CO1CU-B", new PathConstraints(4, 3)));
    autoChooser.addOption("2CO1CU-T", (Command) new Co2Cu1T(builder));
    autoChooser.addOption("2CU-B", (Command) new Cu2B(builder));
    autoChooser.addOption("2CU-M", (Command) new Cu2M(builder));
    //autoChooser.addOption("Score1HighCubeCleanNoBalance", (Command) new Cu1NB(builder));
    autoChooser.addOption("2CU-T", (Command) new Cu2T(builder));

    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  // public void resetPose() {
  //   drivetrain.resetPose(drivetrain.getPose());
  // }
}
