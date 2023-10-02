package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Drivetrain;

public class MoveToTag extends CommandBase {
    private Vision visionSubsystem;
    private Drivetrain drivetrainSubsystem;

    private PIDController horizontalOffsetController;
    private PIDController orientationController;

    private static final double DESIRED_HORIZONTAL_OFFSET = 0.0; // Set your desired horizontal offset in degrees
    private static final double DESIRED_ORIENTATION = 0.0; // Set your desired orientation in degrees
    private static final double HORIZONTAL_OFFSET_TOLERANCE = 2.0; // Set your horizontal offset tolerance in degrees
    private static final double ORIENTATION_TOLERANCE = 2.0; // Set your orientation tolerance in degrees

    private final PIDController distanceController = new PIDController(0,0,0);
    private static final double DESIRED_DISTANCE = 100.0; // Replace with your desired distance
    private static final double YOUR_DISTANCE_TOLERANCE = 2.0; // Replace with your tolerance value
    
    public MoveToTag(Vision visionSubsystem, Drivetrain drivetrainSubsystem) {
        this.visionSubsystem = visionSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;

        
        horizontalOffsetController = new PIDController(0.1, 0.0, 0.0); // Set your PID constants
        orientationController = new PIDController(0.1, 0.0, 0.0); // Set your PID constants

        addRequirements(visionSubsystem, drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        // Set setpoints for the controllers
        horizontalOffsetController.setSetpoint(DESIRED_HORIZONTAL_OFFSET);
        orientationController.setSetpoint(DESIRED_ORIENTATION);

        // Configure tolerances for the controllers
        horizontalOffsetController.setTolerance(HORIZONTAL_OFFSET_TOLERANCE);
        orientationController.setTolerance(ORIENTATION_TOLERANCE);
    }

    @Override
    public void execute() {
        // Get current values from vision subsystem
        double currentHorizontalOffset = visionSubsystem.getHorizontalOffset();
        double currentOrientation = visionSubsystem.getOrientation();

        // Calculate control outputs
        double horizontalOffsetOutput = horizontalOffsetController.calculate(currentHorizontalOffset);
        double orientationOutput = orientationController.calculate(currentOrientation);

        // Use control outputs to drive the robot
        drivetrainSubsystem.drive(0.0, horizontalOffsetOutput, orientationOutput, false);

        // Optional: You can also check if you're close enough to the desired distance
        double currentDistance = visionSubsystem.getDistance();
        double distanceError = currentDistance - DESIRED_DISTANCE;
        double distanceOutput = distanceController.calculate(distanceError);
        boolean distanceWithinTolerance = Math.abs(distanceError) <= YOUR_DISTANCE_TOLERANCE;

        // Check if all conditions are met
        boolean horizontalOffsetWithinTolerance = horizontalOffsetController.atSetpoint();
        boolean orientationWithinTolerance = orientationController.atSetpoint();
        boolean allConditionsMet = horizontalOffsetWithinTolerance && orientationWithinTolerance && distanceWithinTolerance;

        // Print debug information
        System.out.println("Horizontal Offset Error: " + horizontalOffsetController.getPositionError());
        System.out.println("Orientation Error: " + orientationController.getPositionError());
        System.out.println("Distance Error: " + distanceError);

        // Optional: You can also add code to handle this condition, e.g., shooter setup
        if (allConditionsMet) {
            // Add code to handle this condition, e.g., shooter setup
        }
    }

    @Override
    public boolean isFinished() {
        return horizontalOffsetController.atSetpoint() && orientationController.atSetpoint();
    }
}