package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ForkliftConstants;

public class Forklift extends SubsystemBase {
    private final CANSparkMax m_rightForkliftMotor;
    private final CANSparkMax m_leftForkliftMotor;
    private final RelativeEncoder m_rightEncoder;
    private final RelativeEncoder m_leftEncoder;

    private static final ProfiledPIDController m_profiledPIDController = new ProfiledPIDController(
        ForkliftConstants.ProfiledPIDp, 
        ForkliftConstants.ProfiledPIDi, 
        ForkliftConstants.ProfiledPIDd, 
        new TrapezoidProfile.Constraints(
            ForkliftConstants.kForkliftMaxVelocity, 
            ForkliftConstants.kForkliftMaxAcceleration
        )
    );

    public Forklift() {

        m_rightForkliftMotor = new CANSparkMax(ForkliftConstants.rightForkliftChannel, MotorType.kBrushless);
        m_leftForkliftMotor = new CANSparkMax(ForkliftConstants.leftForkliftChannel, MotorType.kBrushless);
 
        m_rightEncoder = m_rightForkliftMotor.getEncoder();
        m_leftEncoder = m_leftForkliftMotor.getEncoder();
    }

    public void stop() {
        m_rightForkliftMotor.stopMotor();
        m_leftForkliftMotor.stopMotor();
    }

    public void moveForklift(double speed) {
        m_rightForkliftMotor.set(speed);
        m_leftForkliftMotor.set(speed);
    }
    
    public double getRightSpeed() {
        return m_rightForkliftMotor.get();
    }
    public double getLeftSpeed() {
        return m_leftForkliftMotor.get();
    }

    public double getRightCounts() {
        return m_rightEncoder.getPosition() / ForkliftConstants.forkliftRatio;
    }
    public double getLeftCounts() {
        return m_leftEncoder.getPosition() / ForkliftConstants.forkliftRatio;
    }

    @Override
    public void periodic() {}

    public void setPosition(double desiredPos) {
        double setpoint = m_profiledPIDController.calculate(getMeasurement(), desiredPos);
        moveForklift(setpoint);
    }

    public double getMeasurement() {
        return getRightCounts();
    }
}