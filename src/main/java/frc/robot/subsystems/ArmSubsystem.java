package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;

//dis is da shih dat don werk
 /* 
public class ArmSubsystem extends SubsystemBase{
    // Motor and encoder initialization
  
   private final SparkMax m_motor = new SparkMax(7, SparkLowLevel.MotorType.kBrushless);
    private final AbsoluteEncoder m_encoder = m_motor.getAbsoluteEncoder(); // For Through-Bore Encoder
    private final SparkMaxConfig armConfig = Configs.Arm.armConfig;

    private final double MIN_OFFSET = 0;
    private final double MAX_OFFSET = Math.PI;

    public ArmSubsystem() {
        // No need to reset position to zero
        m_motor.configure(armConfig, ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        
        // Optionally, you could log or display the initial absolute position
        double initialPosition = m_encoder.getPosition(); // This is the absolute position value
        System.out.println("Initial Encoder Position: " + initialPosition);
    }

    public void controlArm(double value) {
        double motorSpeed = MathUtil.clamp(value, -1, 1);
        double currentPosition = m_encoder.getPosition(); // Gets absolute position directly

        // Scale the current position if necessary based on the gear ratio (if applicable)
        currentPosition /= 16.67; // For example, if using a 5:1 gear ratio

        // Apply limits based on the arm's range of motion
        if (currentPosition >= MAX_OFFSET && motorSpeed > 0) {
            motorSpeed = 0;
        } else if (currentPosition <= MIN_OFFSET && motorSpeed < 0) {
            motorSpeed = 0;
        }

        m_motor.set(motorSpeed);
    }

    public double getEncoderVal() {
        return m_encoder.getPosition() / 16.67; // Adjust for gear ratio if necessary
    }

    @Override
    public void periodic() {
        // Periodically called to update state if needed
    } 
}
     
 */






//bruh dis shih don't work wrong gear ratio head ah ||| I lied + changed gear ratio
//arm starts at encoder ~0 but goes negative when moving up
  public class ArmSubsystem extends SubsystemBase{
   private final SparkMax m_motor = new SparkMax(7, SparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkMaxConfig armConfig = Configs.Arm.armConfig;

    private final double MIN_OFFSET =  -0.503; 
    private final double MAX_OFFSET = .05; 

    private static final double kP = 1.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private final PIDController armPID = new PIDController(kP, kI, kD);

    private double targetPosition = 0.0;

    public ArmSubsystem() {
        m_encoder.setPosition(0);
        m_motor.configure(armConfig, ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public void controlArm(double value) {
        double motorSpeed = MathUtil.clamp(value, -.35, 0.15);
        double currentPosition = m_encoder.getPosition() / 16.67 ; //5:1 ratio
        double armAngle = getArmAngle();  // Convert arm angle to radians
        double kG = 1;
        double holdingVoltage = kG * Math.sin(armAngle * (Math.PI  / 180));  // VALUE * cos(angle)
            //armMotor.setIdleMode(IdleMode.kBrake);


        if (currentPosition >= MAX_OFFSET && motorSpeed > 0) {
            motorSpeed = 0;
        } else if (currentPosition <= MIN_OFFSET && motorSpeed < 0) {
            motorSpeed = 0;
        }

        // If joystick is not being actively used, hold position using feedforward
        if (Math.abs(value) <= 0.05) {
            m_motor.setVoltage(-holdingVoltage);
        } else {
        m_motor.set(motorSpeed); // Normal control
     }
    }



/* 
    //fixed control arm? using set voltage
    public void controlArm(double value) {
        double motorSpeed = MathUtil.clamp(value, -12, 12);
        double currentPosition = m_encoder.getPosition() / 16.67; // 5:1 ratio
        double armAngle = getArmAngleRadians(); // Get angle in radians
        
        double kG = 0.25; // Tune this value for proper gravity compensation
        double gravityCompensationVoltage = kG * Math.sin(armAngle); // Feedforward voltage to hold position

        double appliedVoltage = 0.0;

        if (currentPosition >= MAX_OFFSET && motorSpeed > 0) {
            appliedVoltage = 0.0;
        } else if (currentPosition <= MIN_OFFSET && motorSpeed < 0) {
            appliedVoltage = 0.0;
        } else if (motorSpeed == 0) {
            appliedVoltage = gravityCompensationVoltage; // Apply feedforward voltage to hold position
        } else {
            appliedVoltage = motorSpeed + gravityCompensationVoltage; // Convert speed to voltage and add compensation
        }

        m_motor.setVoltage(appliedVoltage);
    }

*/
     

    public double getEncoderVal(){ //GIVES NEGATIVE VALUE arm is backwards
        return m_encoder.getPosition() / 16.67;
    }
    public double getArmAngle() {
        return -getEncoderVal() * 360.0; //has to be negative bc our encoder is negative
    }
    public double getArmAngleRadians() {
        return Math.toRadians(getArmAngle());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
 

