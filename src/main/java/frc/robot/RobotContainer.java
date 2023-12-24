package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.Balance;
import frc.robot.commands.Arm.LowerArm;
import frc.robot.commands.Arm.RaiseArm;
import frc.robot.commands.Autos.Macros;
import frc.robot.commands.Autos.Routines;
import frc.robot.commands.Drivetrain.FollowTag;
import frc.robot.commands.Drivetrain.JoystickDrive;
import frc.robot.commands.Drivetrain.JoystickDriveSim;
import frc.robot.commands.Drivetrain.MoveToTag;
import frc.robot.commands.Drivetrain.SlowDriveCommunity;
import frc.robot.commands.Drivetrain.TurnToAngle;
import frc.robot.commands.Intake.Intake.IntakeCube;
import frc.robot.commands.Intake.Outtake.OuttakeCube;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DrivetrainSim;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;


public class RobotContainer {

  public final static Drivetrain drivetrain = new Drivetrain();
  public final static DrivetrainSim driveSim = new DrivetrainSim();
  private final Vision vision = new Vision();

  private final static Arm arm = new Arm(ArmConstants.rightArmChannel, ArmConstants.leftArmChannel);
  private final static Intake intake = new Intake(IntakeConstants.cubeIntakeChannel, IntakeConstants.coneIntakeChannel);

  private final static Macros macros = new Macros(arm, intake);

  final Balance balance = new Balance(drivetrain);
  private static SwerveAutoBuilder builder;
  static SendableChooser<List<PathPlannerTrajectory>> autoChooser = new SendableChooser<>();
  static SendableChooser<List<PathPlannerTrajectory>> autoChooserFix = new SendableChooser<>();
  static SendableChooser<Command> autoChooserCommand = new SendableChooser<>();

  private static HashMap<String, Command> eventMap = new HashMap<>();
  private static PPSwerveControllerCommand swerveControllerCommand;

  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private final CommandXboxController m_controller = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final static Routines routines = new Routines(drivetrain, intake, arm, macros);

  public RobotContainer() {
    configureDrivetrainBindings();
    configureArmBindings();
    configureIntakeBindings();
    switch(RobotConstants.getRobot()){
      case ROBOT_2023Cal:
        drivetrain.setDefaultCommand(new JoystickDrive(m_controller, drivetrain, false));
      case ROBOT_SIMBOT:
        driveSim.setDefaultCommand(new JoystickDriveSim(m_controller, driveSim, false));
    }
  }
  
  private void configureDrivetrainBindings() {
    //
    m_controller.b().whileTrue(new MoveToTag(vision, drivetrain));
    m_controller.b().whileTrue(new FollowTag(drivetrain, vision));
    configureIntakeBindings();
    configureArmBindings();
    //m_controller.b().whileTrue(new SlowDriveCommunity(m_controller, drivetrain, true));
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

    autoChooserFix.addOption("MidDock", PathPlanner.loadPathGroup("MidDock", new PathConstraints(4, 3)));
    autoChooserCommand.addOption("Mid Dock", routines.goForward(drivetrain));
    autoChooserCommand.addOption("OutLeftBalance", routines.OutLeftBalance(drivetrain));

    SmartDashboard.putData(autoChooserFix);
  }

  public Command getAutonomousCommand() {
    // PathPlannerTrajectory traj = PathPlanner.loadPath("MidDock",4, 3);
    // return routines.baseSwerveCommand(traj, true)
   //return autoChooserCommand.getSelected();
   return routines.goForward(drivetrain);
  }

  public Command balanceCode(){
    return balance;
  }
}
