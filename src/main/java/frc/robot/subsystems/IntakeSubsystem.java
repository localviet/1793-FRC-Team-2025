package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class IntakeSubsystem extends SubsystemBase{
    private final SparkMax m_motor = new SparkMax(8, SparkLowLevel.MotorType.kBrushless);
    private final SparkMaxConfig armConfig = Configs.Arm.armConfig;


    public IntakeSubsystem() {
        m_motor.configure(armConfig, ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }
    //if rev false, shoot, if rev true, intake
    public void shoot(int rev) {


        double motorSpeed = 0;

        if(rev == 1){
            motorSpeed = 0.3;
        } else if(rev == 2){
            motorSpeed = -0.3;
        } else {
            motorSpeed = 0;
        }

        m_motor.set(motorSpeed);
    }

    public void stop() {
        m_motor.set(0); // Stops the motor
    }


}
