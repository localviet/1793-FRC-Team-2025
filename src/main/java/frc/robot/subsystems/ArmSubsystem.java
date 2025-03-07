package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;

public class ArmSubsystem extends SubsystemBase{
    private final SparkMax m_motor = new SparkMax(5, SparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkMaxConfig armConfig = Configs.Arm.armConfig;

    private final double MIN_OFFSET = 0;
    private final double MAX_OFFSET = Math.PI;

    public ArmSubsystem() {
        m_motor.configure(armConfig, ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public void controlArm(double value) {
        double motorSpeed = value * 0.5;

        double currentPosition = m_encoder.getPosition();

        if (currentPosition >= MAX_OFFSET && motorSpeed > 0) {
            motorSpeed = 0;
        } else if (currentPosition <= MIN_OFFSET && motorSpeed < 0) {
            motorSpeed = 0;
        }

        m_motor.set(motorSpeed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
