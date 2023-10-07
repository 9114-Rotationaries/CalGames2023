package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private edu.wpi.first.cscore.UsbCamera limelightCamera;
    private NetworkTable limelightTable;

    private CameraServer limelightStream;

    // Define constants for network table keys
    private static final String LIMELIGHT_TABLE_NAME = "limelight";
    private static final String TARGET_VALID_KEY = "tv";
    private static final String HORIZONTAL_OFFSET_KEY = "tx";
    private static final String VERTICAL_OFFSET_KEY = "ty";
    private static final String TARGET_AREA_KEY = "ta";

    public Vision() {
        // Initialize the Limelight network table
        limelightTable = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_NAME);

        // Create a new USB camera instance for the Limelight
        limelightCamera = CameraServer.startAutomaticCapture();
        
        // Set camera properties, e.g., brightness
        limelightCamera.setBrightness(0); // Set brightness to 100 (maximum)

        // Set the Limelight LED mode to "off" initially
        limelightTable.getEntry("ledMode").setNumber(1); // 0: Off, 1: On, 2: Blinking

        CameraServer.addCamera(limelightCamera);
        Shuffleboard.getTab("SmartDashboard").add(limelightCamera);
    }

    // Check if Limelight has any valid targets (0 or 1)
    public boolean isTargetValid() {
        return limelightTable.getEntry(TARGET_VALID_KEY).getDouble(0) == 1;
    }

    // Get the horizontal offset (tx) from the crosshair to the target
    public double getHorizontalOffset() {
        return limelightTable.getEntry(HORIZONTAL_OFFSET_KEY).getDouble(0);
    }

    public double getOrientation() {
      // Get the horizontal offset (tx) from the limelight
      NetworkTableEntry txEntry = limelightTable.getEntry("tx");
      double tx = txEntry.getDouble(0.0);

      // Return the horizontal offset as the orientation
      return tx;
  }
    // Get the vertical offset (ty) from the crosshair to the target
    public double getVerticalOffset() {
        return limelightTable.getEntry(VERTICAL_OFFSET_KEY).getDouble(0);
    }

    // Get the target area (ta)
    public double getTargetArea() {
        return limelightTable.getEntry(TARGET_AREA_KEY).getDouble(0);
    }

    public double getDistance() {
      // Get the vertical offset (ty) from the limelight
      NetworkTableEntry tyEntry = limelightTable.getEntry("ty");
      double ty = tyEntry.getDouble(0.0);

      // Calculate the distance based on ty and camera angle
      double angle = Math.toRadians(VisionConstants.cameraAngle + ty);
      double height = VisionConstants.targetHeight - VisionConstants.cameraHeight;
      double distance = height / Math.tan(angle);

      return distance;
  }

    @Override
    public void periodic() {
        // Update values on the SmartDashboard (optional)
        SmartDashboard.putBoolean("Target Valid", isTargetValid());
        SmartDashboard.putNumber("Horizontal Offset (tx)", getHorizontalOffset());
        SmartDashboard.putNumber("Vertical Offset (ty)", getVerticalOffset());
        SmartDashboard.putNumber("Target Area (ta)", getTargetArea());
    }
}