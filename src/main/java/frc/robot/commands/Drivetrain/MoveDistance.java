// MoveDistance.java

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class MoveDistance extends CommandBase {
    private final Drivetrain drivetrain;
    private final PIDController distanceController;
    private final double targetDistance; // Target distance in meters

    public MoveDistance(Drivetrain drivetrain, double targetDistance) {
        this.drivetrain = drivetrain;
        this.targetDistance = targetDistance;
        
        // Create a PID controller for distance control
        distanceController = new PIDController(
            0.1, 0.0, 0.0); // Set your PID constants here
        
        distanceController.setTolerance(0.02); // Set your tolerance value here
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Reset the encoder counts for all modules in the Drivetrain
        drivetrain.resetEncoders();
        
        // Set the setpoint for the distance controller
        distanceController.setSetpoint(targetDistance);
    }

    @Override
    public void execute() {
        // Calculate the current distance using encoder counts and the conversion factor
        double currentDistance = drivetrain.getEncoderCounts() * drivetrain.getDistancePerPulse();
        
        // Calculate the control output using the distance controller
        double output = distanceController.calculate(currentDistance);
        
        // Drive the robot using the calculated output as xSpeed
        drivetrain.drive(output, 0.0, 0.0, false);
    }

    @Override
    public boolean isFinished() {
        // Check if the distance error is within the tolerance
        return distanceController.atSetpoint();
    }
}
