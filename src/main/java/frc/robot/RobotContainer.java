// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Hashtable;
import java.util.List;

import javax.swing.plaf.TreeUI;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.Drivetrain.JoystickDrive;
import frc.robot.commands.Drivetrain.MoveToTag;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Balance;
import frc.robot.commands.Arm.LowerArm;
import frc.robot.commands.Arm.RaiseArm;
import frc.robot.commands.Autos.Routines;
import frc.robot.commands.Intake.Intake.IntakeCone;
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

  public final static Drivetrain drivetrain = new Drivetrain();
  private final Vision vision = new Vision();

  private final Arm arm = new Arm(ArmConstants.rightArmChannel, ArmConstants.leftArmChannel);
  private final static Intake intake = new Intake(IntakeConstants.cubeIntakeChannel, IntakeConstants.coneIntakeChannel);

  final Balance balance = new Balance(drivetrain);
  private static SwerveAutoBuilder builder;
  static SendableChooser<List<PathPlannerTrajectory>> autoChooser = new SendableChooser<>();
  static SendableChooser<Command> autoChooserFix = new SendableChooser<>();

  private static HashMap<String, Command> eventMap = new HashMap<>();
  private static PPSwerveControllerCommand swerveControllerCommand;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private final CommandXboxController m_controller = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final static Routines routines = new Routines(drivetrain);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureDrivetrainBindings();
    drivetrain.setDefaultCommand(new JoystickDrive(m_controller, drivetrain, true));

    configureArmBindings();
    configureIntakeBindings();
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

  

  public void setUpAutos(){
    //autoChooser.setDefaultOption("Do Nothing", new WaitCommand(15));
    // autoChooser.addOption("Go Forward", PathPlanner.loadPathGroup("Go Forward", new PathConstraints(1, 2)));   
    // autoChooser.addOption("1CO1CU-M", PathPlanner.loadPathGroup("1CO1CU-M", new PathConstraints(4, 3)));
    // autoChooser.addOption("1CO1CU-B", PathPlanner.loadPathGroup("1CO1CU-B", new PathConstraints(4, 3)));
    // autoChooser.addOption("1CO1CU-T", PathPlanner.loadPathGroup("1CO1CU-T", new PathConstraints(4, 3)));
    // autoChooser.addOption("2CO-B", PathPlanner.loadPathGroup("2CO-B", new PathConstraints(10, 6)));
    // autoChooser.addOption("2CO-M", PathPlanner.loadPathGroup("2CO-M", new PathConstraints(4, 3)));
    // autoChooser.addOption("2CO-T", PathPlanner.loadPathGroup("2CO-T", new PathConstraints(4, 3)));
    // autoChooser.addOption("2CO1CU-B", PathPlanner.loadPathGroup("2CO1CU-B", new PathConstraints(4, 3)));
    // autoChooser.addOption("2CO1CU-M", PathPlanner.loadPathGroup("2CO1CU-M", new PathConstraints(4, 3)));
    // autoChooser.addOption("2CO1CU-T", PathPlanner.loadPathGroup("2CO1CU-T", new PathConstraints(4, 3)));
    // autoChooser.addOption("2CU-B", PathPlanner.loadPathGroup("2CU-B", new PathConstraints(4, 3)));
    // autoChooser.addOption("2CU-M", PathPlanner.loadPathGroup("2CU-M", new PathConstraints(4, 3)));
    // autoChooser.addOption("2CU-T", PathPlanner.loadPathGroup("2CU-T", new PathConstraints(4, 3)));
    // autoChooser.addOption("test", PathPlanner.loadPathGroup("test", new PathConstraints(4, 3)));
    // autoChooser.addOption("PathWithStuff", PathPlanner.loadPathGroup("PathWithStuff", new PathConstraints(4, 3)));
    // autoChooser.addOption("SwerveTest", PathPlanner.loadPathGroup("TestingSwerve", new PathConstraints(4, 3)));


    autoChooserFix.addOption("Go Forward Fix", routines.goForward(drivetrain));

    eventMap.put("event", new PrintCommand("Passed marker 1"));
    //eventMap.put("Balance",  balance);
    eventMap.put("OutCube", new InstantCommand(intake::cOut, intake));
    eventMap.put("InCube", new InstantCommand(intake::cInt, intake)); //cInt = cubeIntake without needing parameter
    eventMap.put("ArmUp", new InstantCommand(arm::armUp, arm)); //cInt = cubeIntake without needing parameter
    eventMap.put("ArmDown", new InstantCommand(arm::armDown, arm)); //down is up and up is down


    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return routines.goForward(drivetrain);
    }

  public Command balanceCode(){
  return balance;
    //CommandScheduler.getInstance().schedule(balance); 
  }
  // public void resetPose() {
  //   drivetrain.resetPose(drivetrain.getPose());
  // }
}
