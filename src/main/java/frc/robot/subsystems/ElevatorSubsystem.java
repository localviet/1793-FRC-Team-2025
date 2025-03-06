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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    // Sensors
    /*    private final LaserCan m_elevatorLaserCan = new LaserCan(0);
    private final LaserCanSim m_elevatorLaserCanSim = new LaserCanSim(0);
    private final RegionOfInterest m_laserCanROI = new RegionOfInterest(0, 0, 16, 16);
    private final TimingBudget m_laserCanTimingBudget = TimingBudget.TIMING_BUDGET_20MS;
     * 
     */

    private final Alert m_laserCanFailure = new Alert("LaserCAN failed to configure.",
            AlertType.kError);
    private final DigitalInput m_limitSwitchLow = new DigitalInput(9);
    private DIOSim m_limitSwitchLowSim = null;


    public static Distance convertRotationsToDistance(Angle rotations){
      return Meters.of(rotations.in(Rotations) *
              (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius) / ElevatorConstants.kElevatorGearing);
    }

    public static Angle convertDistanceToRotations(Distance distance){
      return Rotations.of(distance.in(Meters) /
              (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius) * ElevatorConstants.kElevatorGearing);
    }


    public ElevatorSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(40)
                .openLoopRampRate(ElevatorConstants.kElevatorRampRate);
        
        m_motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);//




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
        double voltsOutput = MathUtil.clamp(
                m_feedForward.calculateWithVelocities(getVelocityMetersPerSecond(), m_controller.getSetpoint().velocity)
                + m_controller.calculate(getPositionMeters(), goal),
                -7,
                7);
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

