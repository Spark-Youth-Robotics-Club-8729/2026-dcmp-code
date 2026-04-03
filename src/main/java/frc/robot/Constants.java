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
  public static final class ShooterConstants {
    // CAN IDs
    public static final int leftFlywheelID = 10;
    public static final int rightFlywheelID = 11;
    public static final int hoodMotorID = 12;
    public static final int feederMotorID = 13; // renamed from indexerMotorID — green feeder wheels
    public static final String canBus = "rio"; // Change to "canivore" if using CANivore

    // ---------------------------------------------------------------------------
    // Gear ratios
    // ---------------------------------------------------------------------------
    // Flywheel: 1:1 direct drive
    public static final double flywheelGearRatio = 1.0;

    // Hood pivot: 13.7778:1  (13.7778 motor rotations = 1 degree of hood travel)
    // TalonFX SensorToMechanismRatio expects motor-rotations / mechanism-rotations.
    // 13.7778 motor degrees per hood degree = 13.7778 motor rotations per hood rotation.
    public static final double hoodGearRatio = 13.7778;

    // Feeder (green wheels, hopper→shooter): 1:1.375  (1 motor rotation = 1.375 mechanism rotations)
    // Stored as motor-rotations / mechanism-rotations for SensorToMechanismRatio.
    public static final double feederGearRatio = 1.0 / 1.375; // ≈ 0.7273

    // ---------------------------------------------------------------------------
    // Flywheel physical constants
    // ---------------------------------------------------------------------------
    // Combined MOI of flywheel assembly (flywheel, shooter wheels, shaft, pulleys)
    // 16.436 in²·lb  →  kg·m²
    public static final double flywheelMOI =
        Units.lbsToKilograms(16.436 * Units.inchesToMeters(1.0) * Units.inchesToMeters(1.0));
    // Equivalent: 16.436 * 0.000292639 ≈ 0.004809 kg·m²

    // Radius of the flywheel wheel that contacts the note (used for physics-based RPM calculation).
    // TODO: Set to the actual shooter wheel radius.
    public static final double flywheelWheelRadiusM = Units.inchesToMeters(2.0);

    // Height of the shooter exit point above the floor (used for projectile Δh calculation).
    // TODO: Measure on the actual robot.
    public static final double shooterExitHeightM = Units.inchesToMeters(24.0);

    // Fraction of flywheel surface speed transferred to the note exit speed.
    // Accounts for wheel compression / slip (typical range: 0.80 – 0.95).

    // ---------------------------------------------------------------------------
    // Flywheel PID / feedforward constants
    // ---------------------------------------------------------------------------
    public static final double flywheelKp = 0.4;
    public static final double flywheelKi = 0.0;
    public static final double flywheelKd = 0.0;
    public static final double flywheelKv = 0.12; // Volts per RPS
    public static final double flywheelKs = 0.25; // Static friction — helps hold speed under load

    // ---------------------------------------------------------------------------
    // Hood PID constants
    // ---------------------------------------------------------------------------
    public static final double hoodKp = 24.0; // Increased — 1.0 may be too weak against gravity/load
    public static final double hoodKi = 0.0;
    public static final double hoodKd = 0.3; // Small D to damp oscillation

    // ---------------------------------------------------------------------------
    // Feeder (green wheels, hopper→shooter) PID / feedforward constants
    // ---------------------------------------------------------------------------
    public static final double feederKp = 0.1; // TODO: Tune
    public static final double feederKi = 0.0;
    public static final double feederKd = 0.0;
    public static final double feederKv = 0.12;

    // ---------------------------------------------------------------------------
    // Hood angle limits (radians, at the mechanism — after gear reduction)
    // ---------------------------------------------------------------------------
    public static final double hoodMinAngleRad = Units.degreesToRadians(5.0); // TODO: Set actual
    public static final double hoodMaxAngleRad = Units.degreesToRadians(45.0); // TODO: Set actual

    // ---------------------------------------------------------------------------
    // Tolerances
    // ---------------------------------------------------------------------------
    public static final double flywheelToleranceRPM = 100; // widened 50 to 100 RPM while tuning
    public static final double hoodToleranceRad =
        Units.degreesToRadians(5.0); // Widened from 3° to 5° while tuning

    // ---------------------------------------------------------------------------
    // Default / feeder speeds
    // ---------------------------------------------------------------------------
    public static final double defaultFlywheelSpeedRPM = 872.9; // TODO: changeback to more
    public static final double maxFlywheelSpeedRPM = 6000.0;

    // Positive = feeds ball up from hopper to shooter
    public static final double feederFeedSpeedRPM = 800.0;
    // Negative = ejects ball back down toward hopper
    public static final double feederEjectSpeedRPM = -500.0;
  }
  public static final class intakeconstants{
    public static final int kRollerID = 20;
    public static final int kSlapdownID = 21;
    public static final double slapdowngearratio = 45;
    // Slapdown angle limits (radians, at the mechanism after gear reduction)
    public static final double slapdownUpAngleRad = 0; // TODO: Set actual
    public static final double slapdownDownAngleRad =
        1.6; // TODO: Set actual

    // Slapdown PID gains (separate for each mode so they can be tuned independently)
    public static final double slapdownUpKp = 0.5; // TODO: Tune
    public static final double slapdownUpKd = 0.15; // TODO: Tune

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