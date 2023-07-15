// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;




public class Vision extends SubsystemBase {
  /** Creates a new Vision. */


  private NetworkTable limelightTable;
  private double tx;
  private double ty;
  private double ta;
  private double ts;

  public Vision() {
    
    UsbCamera limelight = CameraServer.startAutomaticCapture();
    limelight.setBrightness(20);

    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
   // tx = limelightTable.getEntry("tx");
   // ty = limelightTable.getEntry("ty");
   // ta = limelightTable.getEntry("ta");
    

  }

  public double getTX(){
    limelightTable.getEntry("pipeline").setNumber(0);
    tx = limelightTable.getEntry("tx").getDouble(0.0);
    return tx;
  }

  public double getTY(){
    limelightTable.getEntry("pipeline").setNumber(0);
    ty = limelightTable.getEntry("ty").getDouble(0.0);
    return ty;
  }

  public double getTA(){
    limelightTable.getEntry("pipeline").setNumber(0);
    ta = limelightTable.getEntry("ta").getDouble(0.0);
    return ta;
  }

  public double getDistance(){
    double angle = Math.toRadians(VisionConstants.cameraAngle + ty);
    double height = VisionConstants.targetHeight - VisionConstants.cameraHeight;
    double distance = height / Math.tan(angle);
    return distance;
  }

  public double getTS(){
    limelightTable.getEntry("pipelines").setNumber(0);
    ts = limelightTable.getEntry("ts0").getDouble(0);
    return ts;
  }

  public double getTV(){
    double[] tcornxy = limelightTable.getEntry("tcornxy").getDoubleArray(new double[8]);

    if (tcornxy.length >= 8) {
      double x0 = tcornxy[0];
      double y0 = tcornxy[1];
      double x1 = tcornxy[2];
      double y1 = tcornxy[3];
      double x2 = tcornxy[4];
      double y2 = tcornxy[5];
      double x3 = tcornxy[6];
      double y3 = tcornxy[7];

      double leftSlope = (y2 - y0) / (x2 - x0); //Tag Left edge slope 
      double rightSlope = (y3 - y1) / (x3 - x1); //Tag Right edge slope
    
      double avgSlope = (leftSlope + rightSlope) / 2; //Tag avgS slope

      double tilt = Math.toDegrees(Math.atan(avgSlope));

      return tilt;
    } else {
      return 0;
    }

    
    
  }

  public AprilTagDetection[] getAprilTag(){
    AprilTagDetector detector = new AprilTagDetector();

    NetworkTableEntry imageEntry = limelightTable.getEntry("image");
    byte[] imageBytes = imageEntry.getRaw(new byte[0]);
    Mat image = Imgcodecs.imdecode(new MatOfByte(imageBytes),Imgcodecs.IMREAD_UNCHANGED); //flags???

    return detector.detect(image);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // double x = tx.getDouble(0.0);
    // double y = ty.getDouble(0.0);
    // double area = ta.getDouble(0.0);

    // SmartDashboard.putNumber("LimelightX", x);
    // SmartDashboard.putNumber("LimelightY", y);
    // SmartDashboard.putNumber("LimelightArea", area);
  }
}
