package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ForkliftConstants;

public class Forklift extends SubsystemBase {
    private final CANSparkMax m_rightForkliftMotor;
    private final CANSparkMax m_leftForkliftMotor;
    private final RelativeEncoder m_rightEncoder;
    private final RelativeEncoder m_leftEncoder;

    public Forklift() {
        //Defining motors
        m_rightForkliftMotor = new CANSparkMax(ForkliftConstants.rightForkliftChannel, MotorType.kBrushless);
        m_leftForkliftMotor = new CANSparkMax(ForkliftConstants.leftForkliftChannel, MotorType.kBrushless);
 
        //Defining encoders
        m_rightEncoder = m_rightForkliftMotor.getEncoder();
        m_leftEncoder = m_leftForkliftMotor.getEncoder();
    }

    //Stops motor
    public void stop() {
        m_rightForkliftMotor.stopMotor();
        m_leftForkliftMotor.stopMotor();
    }

    //Sets the speed of the motor
    //Moves the forklift up & down.
    public void moveForklift(double speed) {
        m_rightForkliftMotor.set(speed);
        m_leftForkliftMotor.set(speed);
    }
    
    //Returns speed of the motors (L & R)
    public double getRightSpeed() {
        return m_rightForkliftMotor.get();
    }
    public double getLeftSpeed() {
        return m_leftForkliftMotor.get();
    }


    //Returns how many times the larger of the two gears rotates 
    public double getRightCounts() {
        return m_rightEncoder.getPosition() / ForkliftConstants.forkliftRatio;
    }
    public double getLeftCounts() {
        return m_leftEncoder.getPosition() / ForkliftConstants.forkliftRatio;
    }
    
    //Prints all the speeds and counts.
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Right Forklift Motor Speed", getRightSpeed());
        SmartDashboard.putNumber("Left Forklift Motor Speed", getLeftSpeed());
        SmartDashboard.putNumber("Right Encoder Position", getRightCounts());
        SmartDashboard.putNumber("Left Encoder Position", getLeftCounts());
    }




}
