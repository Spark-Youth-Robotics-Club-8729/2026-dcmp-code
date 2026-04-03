// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
    public static final double kMaxSpeedMetersPerSecond = 0.1;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    
    public static final double kP = 1.4;
    public static final double kI = 0.4;
    public static final double kD = 0.2;

    public static final double kPSnap = 0.02;
    public static final double kISnap = 0.0;
    public static final double kDSnap = 0.002;
    public static final double snapTolerance = 10.0; //degrees

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs (edited to ours)

    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 4;
    public static final int kFrontRightDrivingCanId = 6;
    public static final int kRearRightDrivingCanId = 8;

    public static final int kFrontLeftTurningCanId = 1;
    public static final int kRearLeftTurningCanId = 3;
    public static final int kFrontRightTurningCanId = 5;
    public static final int kRearRightTurningCanId = 7;

    public static final boolean kGyroReversed = false;
  }
  public static final class indexerconstants{

    public static final int kIndexerCanId = 22;
    public static final double gearratio = 4.0;
    public static final double feedVolts = 6.0;
    public static final double reverseVolts = -4.0;
  }
  public static final class intakeconstants{
    public static final int kRollerID = 20;
    public static final int kSlapdownID = 21;
    public static final double slapdowngearratio = 45;
    // Slapdown angle limits (radians, at the mechanism after gear reduction)
    public static final double slapdownUpAngleRad = Units.degreesToRadians(0.0); // TODO: Set actual
    public static final double slapdownDownAngleRad =
        Units.degreesToRadians(90.0); // TODO: Set actual

    // Slapdown PID gains (separate for each mode so they can be tuned independently)
    public static final double slapdownUpKp = 0.3; // TODO: Tune
    public static final double slapdownUpKd = 0.4; // TODO: Tune

    public static final double slapdownDownKp = 0.15; // TODO: Tune
    public static final double slapdownDownKd = 0.8; // TODO: Tune

    public static final double slapdownJitterKp = 0.5; // TODO: Tune
    public static final double slapdownJitterKd = 0.5; // TODO: Tune

    // Slapdown tolerance
    public static final double slapdownToleranceRad = Units.degreesToRadians(5.0);

    // Slapdown stall detection (stop PID if we likely hit the bumper)
    public static final double slapdownStallCurrentAmps = 25.0; // above normal running current
    public static final double slapdownStallAppliedVolts = 8.0; // near full output
    public static final double slapdownStallVelocityRadPerSec = Units.degreesToRadians(5.0);
    public static final double slapdownStallDebounceSec = 0.15;

    // Roller voltages (positive = intake in, negative = outtake)
    public static final double rollerIntakeVolts = 6.0; // Reduced from 10.0 — TODO: Tune
    public static final double rollerOuttakeVolts = -4.0; // Reduced from -6.0 — TODO: Tune

    // Jitter constans
    public static final double jitterFrequencyHz = 0.25; // Tune
    public static final double jitterAmplitudeDeg = 45.0; // Tune


    }
    public static final class ModuleConstants {
      // The MAXSwerve module can be configured with one of three pinion gears: 12T,
      // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
      // more teeth will result in a robot that drives faster).
      public static final int kDrivingMotorPinionTeeth = 12;

      // Calculations required for driving motor conversion factors and feed forward
      public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
      public static final double kWheelDiameterMeters = 0.0762;
      public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
      // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
      // teeth on the bevel pinion
      public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
      public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
          / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.3;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 2;
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

  
  

}