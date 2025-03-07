package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
//import frc.robot.Constants.AlgaeArmConstants;
import edu. wpi. first. math. trajectory. TrapezoidProfile.Constraints;
//import frc.robot.RobotMath.Elevator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;


public class BargeSubsystem extends SubsystemBase{
    private final SparkMax m_motor = new SparkMax(Constants.BargeConstants.bargeID, SparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
;
    private final SparkMaxConfig config = Configs.Barge.bargeConfig;

    private final double maxOffset = -21;
    private final double restOffset = -1;

    private boolean clamped = false;

    public BargeSubsystem() {

        m_motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    }

    public double getVelocityMetersPerSecond() {
        double winchRadius = Constants.BargeConstants.winchRadi; // Replace with actual winch radius in meters
        double gearRatio = Constants.BargeConstants.gearRatio; // 10:1 * 10:1 = 100:1
    
        return (m_encoder.getVelocity() / 60) * (2 * Math.PI * winchRadius) / gearRatio;
    }
    public double getPositionMeters() {
        double winchRadius = Constants.BargeConstants.winchRadi;
        double gearRatio = Constants.BargeConstants.gearRatio;

        // Convert encoder rotations to actual winch displacement in meters
        return (m_encoder.getPosition() / gearRatio) * (2 * Math.PI * winchRadius);
    }
    public void reachGoal(double goal) {
        // Get current position
        double currentOffset = m_encoder.getPosition(); 
        
        // Calculate error (how far we are from the target)
        double error = goal - currentOffset;
        
        // Simple proportional control (P-Control) for correction
        double kP = 1.5; // Tune this value for better control
        double output = kP * error; 
        
        // Add optional feedforward for gravity/friction compensation
        double feedForwardVolts = 0;
        
        // Combine control output and feedforward
        double voltsOutput = MathUtil.clamp(output + feedForwardVolts, -12, 12);
        
        // Apply voltage to the motor
        m_motor.setVoltage(voltsOutput);
    }

    public Command setClampPosition(boolean clamped) {
        double targetOffset = clamped ? maxOffset : restOffset;
        
        // Continuously apply reachGoal() until the command is interrupted
        return new RunCommand(() -> reachGoal(targetOffset), this);
    }


    public void toggleClamp() {
        clamped = !clamped;
        setClampPosition(clamped).schedule(); // Flip the state and run command
    }

    public double getEncoderVal(){
        return m_encoder.getPosition();
      }
}
