// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4; //4.8 before
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = 2.0354;
    public static final double kFrontRightChassisAngularOffset = 1.20;
    public static final double kBackLeftChassisAngularOffset = 0.9007;
    public static final double kBackRightChassisAngularOffset = 2.286;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 13;
    public static final int kRearLeftDrivingCanId = 14;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 1;

    public static final int kFrontLeftTurningCanId = 12;
    public static final int kRearLeftTurningCanId = 15;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearRightTurningCanId = 4;  //was zero

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3); //0.0762 before
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.15;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

    public static class ElevatorConstants {
    public static final double kElevatorKp = 5;//5
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0;//
    public static final double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
    public static final double kMaxAcceleration = Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);
    public static final double kElevatorkS = 0.02;
    public static final double kElevatorkG = 0.9;
    public static final double kElevatorkV = 3.8;
    public static final double kElevatorkA = 0.17;
    public static final double kElevatorRampRate = 0.1;
    public static final double kElevatorGearing = 12.0; // gear is Drum Teeth / Motor Teeth || pully is diameter of Drum / dia of Motor
    public static final double kElevatorCarriageMass = 4.0;
    public static final double kElevatorDrumRadius = 0.05; //radius of elevator gear 5cm
    public static final double kElevatorMinHeightMeters = 0.155; //CHANGE THESE min height is from reference point to bottom of elevator
    public static final double kElevatorMaxHeightMeters = 1.59; // max is from reference to the top of the elevator
    public static final double kElevatorLength = Inches.of(62.6).in(Meters); //from bottom to very top
    public static final Distance kElevatorStartingHeightSim = Meters.of(0.0);
    public static final Angle kElevatorStartingAngle = Degrees.of(-90);
    public static final Distance kLaserCANOffset  = Inches.of(3);
    public static final double kElevatorDefaultTolerance = Inches.of(1).in(Meters);

    public static double kLowerToScoreHeight =  Units.inchesToMeters(6);;
  }

  public static class BargeConstants{

     public static final int bargeID = 6;

     public static final double winchRadi = 0.02; //in meters
     public static final double gearRatio = 100;

  }
}
