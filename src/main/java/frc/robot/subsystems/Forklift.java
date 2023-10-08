package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ForkliftConstants;
import frc.robot.Constants.SwerveConstants;

public class Forklift extends ProfiledPIDSubsystem {
    private final CANSparkMax m_rightForkliftMotor;
    private final CANSparkMax m_leftForkliftMotor;
    private final RelativeEncoder m_rightEncoder;
    private final RelativeEncoder m_leftEncoder;

    private static final ProfiledPIDController m_profiledPIDController = new ProfiledPIDController(
        SwerveConstants.ProfiledPIDp, 
        SwerveConstants.ProfiledPIDi, 
        SwerveConstants.ProfiledPIDd, 
        new TrapezoidProfile.Constraints(
            SwerveConstants.kModuleMaxAngularVelocity, 
            SwerveConstants.kModuleMaxAngularAcceleration
        )
    );

    //private final ProfiledPIDSubsystem m_motionProfiler;
    public Forklift() {
        super(
            m_profiledPIDController
        );

        //Defining motors
        m_rightForkliftMotor = new CANSparkMax(ForkliftConstants.rightForkliftChannel, MotorType.kBrushless);
        m_leftForkliftMotor = new CANSparkMax(ForkliftConstants.leftForkliftChannel, MotorType.kBrushless);
 
        //Defining encoders
        m_rightEncoder = m_rightForkliftMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        m_leftEncoder = m_leftForkliftMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
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

    public void setPosition(double desiredPos) {
        double setpoint = m_controller.calculate(getMeasurement(), desiredPos);
        moveForklift(setpoint);
    }

    public double getMeasurement() {
        return getRightCounts()*360;
    }

    public void useOutput(double output, State setpoint) {
    }
}
