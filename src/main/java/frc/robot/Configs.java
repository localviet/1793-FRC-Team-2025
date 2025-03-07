package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(45); //40-45
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20); //20-30
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.35, 0, .7)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }


    public static final class Elevator {
        public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();

        static {
            // Define the PID configuration for the elevator motor
            elevatorConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(40)
                    .openLoopRampRate(ElevatorConstants.kElevatorRampRate);
            elevatorConfig.closedLoop
                    .pid(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi, ElevatorConstants.kElevatorKd)
                    .velocityFF(ElevatorConstants.kElevatorkG) // Feedforward term
                    .outputRange(-7, 7);
        }
    }

    public static final class Barge {
        public static final SparkMaxConfig bargeConfig = new SparkMaxConfig();

        static {
                // Define the PID configuration for the elevator motor
                bargeConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(40);
                bargeConfig.closedLoop
                        .pid(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi, ElevatorConstants.kElevatorKd)
                        .velocityFF(ElevatorConstants.kElevatorkG) // Feedforward term
                        .outputRange(-12, 12);
            }
        }

    public static final class Arm {
        public static final SparkMaxConfig armConfig = new SparkMaxConfig();

        static {
                // Define the PID configuration for the elevator motor
                armConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(40)
                        .openLoopRampRate(ElevatorConstants.kElevatorRampRate);
                armConfig.closedLoop
                        .pid(.5, ElevatorConstants.kElevatorKi, ElevatorConstants.kElevatorKd)
                        .velocityFF(ElevatorConstants.kElevatorkG) // Feedforward term
                        .outputRange(-12, 12);
        }
    }

}


