package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;

public class CheckDistance extends CommandBase { 
    public CheckDistance(Vision vision) { // Pass the Vision instance to the constructor
        // You can initialize other variables or do additional setup here if needed.
        // limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void checkDistance() {
        // Check distance to charging station using Limelight
        // Make sure you have access to the 'limelightTable' instance here.
        double distance = Vision.limelightTable.getEntry("distance").getDouble(0.0); // Replace "distance" with the actual Limelight key

        double targetDistance = 10.0; // Adjust this value as needed
        double error = targetDistance - distance;

        if (error > 0.1) {
            // Notify that it is too far.
            SmartDashboard.putNumber("Too far by: ", error);
        } else if (error < -0.1) {
            // Notify that it is too close
            SmartDashboard.putNumber("Too close by: ", error);
        } else {
            // Notify that ready for forklift down.
            SmartDashboard.putNumber("Ready to dock | Error is: ", error);
        }
    }
}
