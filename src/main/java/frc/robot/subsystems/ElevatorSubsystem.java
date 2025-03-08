package frc.robot.subsystems;

/*
 * import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
 */

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
//import frc.robot.Constants.AlgaeArmConstants;
import edu. wpi. first. math. trajectory. TrapezoidProfile.Constraints;
//import frc.robot.RobotMath.Elevator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

//import static au.grapplerobotics.interfaces.LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT;
import static edu.wpi.first.units.Units.*;

public class ElevatorSubsystem extends SubsystemBase {
    //setUp

    private final SparkMax m_motor = new SparkMax(5, SparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final ElevatorFeedforward m_feedForward = new ElevatorFeedforward(ElevatorConstants.kElevatorkS,
            ElevatorConstants.kElevatorkG,
            ElevatorConstants.kElevatorkV,
            ElevatorConstants.kElevatorkA);
     
    private final SparkMaxConfig config = Configs.Elevator.elevatorConfig;

    //private final DigitalInput m_limitSwitchLow = new DigitalInput(9); //do we have limit switch?



    public ElevatorSubsystem() {
        m_motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }



    public double getPositionMeters() {
        return m_encoder.getPosition() * (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius)
                / ElevatorConstants.kElevatorGearing;
    }

    public double getVelocityMetersPerSecond() {
        return (m_encoder.getVelocity() / 60) * (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius)
                / ElevatorConstants.kElevatorGearing;
    }

    public void reachGoal(double goal){


        double error = goal - getPositionMeters(); // Distance error
        double kP = Constants.ElevatorConstants.kElevatorKp; // Tune this for better control
        double output = kP * error;
    
        // Feedforward for gravity/friction compensation, thats it
        double feedForwardVolts = m_feedForward.calculateWithVelocities(getVelocityMetersPerSecond(), 0);
        
        // Combine control output and feedforward, then clamp voltage
        double voltsOutput = MathUtil.clamp(output + feedForwardVolts, -12, 12);
        
        m_motor.setVoltage(voltsOutput);

    }
    

    public Command setGoal(double goal){
        return run(() -> reachGoal(goal));
    }

    public Command setElevatorHeight(double height){
        return setGoal(height).until(()->aroundHeight(height));
    }

    public boolean aroundHeight(double height){
        return aroundHeight(height, ElevatorConstants.kElevatorDefaultTolerance);
    }
    public boolean aroundHeight(double height, double tolerance){
        return MathUtil.isNear(height,getPositionMeters(),tolerance);
    }

     /**
     * Stop the control loop and motor output.
     */
    public void stop()
    {
        m_motor.set(0.0);
    }

    /**
     * Update telemetry, including the mechanism visualization.
     */
    public void updateTelemetry()
    {
    }

    @Override
    public void periodic()
    {
    }
}

