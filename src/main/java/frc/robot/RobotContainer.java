// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShotCalculator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.NetworkValues;
import java.util.List;

import frc.robot.command.SystemTestCommand;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(
      () -> Rotation2d.fromDegrees(m_robotDrive.getHeading()), null);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  boolean m_fieldRelative = true;

  private final PIDController m_snapController = new PIDController(
      DriveConstants.kPSnap, DriveConstants.kISnap, DriveConstants.kDSnap);

  private double speed_Offense_Defense = 1.0;
  private final double offense_speed = 1.0;
  private final double defense_speed = 1.7;

  // FIX: Declared autoChooser as a class field
  private final SendableChooser<Command> autoChooser;

  public DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Initialize shot calculator
    ShotCalculator.initialize(
        m_robotDrive::getPose,
        m_robotDrive::getVelocity,
        m_visionSubsystem.getAllianceHubPosition());

    SmartDashboard.putNumber("Robot Gyro", m_robotDrive.getHeading());

    // FIX: registerNamedCommands() is now defined below and called here in the constructor
    registerNamedCommands();

    // FIX: autoChooser setup moved into the constructor where it belongs
    autoChooser = AutoBuilder.buildAutoChooser();

    // Add every .path file as a standalone auto option
    String[] pathNames = {
        // Starting position paths
        "Red Middle Preload Shoot", "Red 2 Cycle Right", "Red 2 Cycle Left", "HP Red Auto", "HP Blue Auto", "Blue One Cycle Left",
        "Blue Middle HP", "Blue 2 Cycle Right",
        "Blue 2 Cycle Left", "Blue Middle Preload Shoot", "Blue Leave Middle"
    };

    for (String pathName : pathNames) {
      try {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        autoChooser.addOption("Path: " + pathName, AutoBuilder.followPath(path));
      } catch (Exception e) {
        System.err.println("Failed to load path: " + pathName);
      }
    }

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband) * speed_Offense_Defense,
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband) * speed_Offense_Defense,
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                m_fieldRelative),
            m_robotDrive));
  }

  /**
   * FIX: Added registerNamedCommands() method that was called but never defined.
   * Register any PathPlanner named commands here that are used in .auto files.
   */
private void registerNamedCommands() {

    // Feed note through shooter + indexer
    NamedCommands.registerCommand(
        "ShooterFeed",
        Commands.startEnd(
            () -> {
              m_shooterSubsystem.feedNote();
              m_indexerSubsystem.index();
            },
            () -> {
              m_shooterSubsystem.stopFeeder();
              m_indexerSubsystem.stopindexer();
            },
            m_shooterSubsystem,
            m_indexerSubsystem));

    // Stop shooter and indexer completely
    NamedCommands.registerCommand(
        "ShooterStop",
        Commands.runOnce(
            () -> {
                m_shooterSubsystem.setFlywheelVelocities(0, 0);
                m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMinAngleRad);
                m_shooterSubsystem.stopFeeder();
                m_indexerSubsystem.stopindexer();
                ShotCalculator.getInstance().clearParameters();
            },
            m_shooterSubsystem,
            m_indexerSubsystem));

    // Lock wheels in X formation (replaces drive::stopWithX)
    NamedCommands.registerCommand(
        "DriveStopWithX",
        Commands.runOnce(m_robotDrive::setX, m_robotDrive));

    // Intake in
    NamedCommands.registerCommand(
        "IntakeIN",
        Commands.startEnd(
            () -> m_intakeSubsystem.intake(),
            () -> m_intakeSubsystem.stopintake(),
            m_intakeSubsystem));

    NamedCommands.registerCommand(
        "IntakeIn",
        Commands.startEnd(
            () -> m_intakeSubsystem.intake(),
            () -> m_intakeSubsystem.stopintake(),
            m_intakeSubsystem));

    // Intake out
    NamedCommands.registerCommand(
        "IntakeOUT",
        Commands.startEnd(
            () -> m_intakeSubsystem.outtake(),
            () -> m_intakeSubsystem.stopintake(),
            m_intakeSubsystem));

    NamedCommands.registerCommand(
        "IntakeOut",
        Commands.startEnd(
            () -> m_intakeSubsystem.outtake(),
            () -> m_intakeSubsystem.stopintake(),
            m_intakeSubsystem));

    // Slapdown down
    NamedCommands.registerCommand(
        "IntakeSlapdownDownOnly",
        Commands.runOnce(
            () -> m_intakeSubsystem.slapdowndown(),
            m_intakeSubsystem));

    // Slapdown up (retract)
    NamedCommands.registerCommand(
        "IntakeRetract",
        Commands.runOnce(
            () -> m_intakeSubsystem.slapdownup(),
            m_intakeSubsystem));


    // Slapdown down then intake (deploy + run rollers)
    NamedCommands.registerCommand(
        "SlapdownAndIntakeCommand",
        Commands.sequence(
            Commands.runOnce(() -> m_intakeSubsystem.slapdowndown(), m_intakeSubsystem),
            Commands.startEnd(
                () -> m_intakeSubsystem.intake(),
                () -> m_intakeSubsystem.stopintake(),
                m_intakeSubsystem)));

    NamedCommands.registerCommand(
        "StartIntakeSlapdown",
        Commands.sequence(
            Commands.runOnce(() -> m_intakeSubsystem.slapdowndown(), m_intakeSubsystem),
            Commands.startEnd(
                () -> m_intakeSubsystem.intake(),
                () -> m_intakeSubsystem.stopintake(),
                m_intakeSubsystem)));

    // Indexer commands
    NamedCommands.registerCommand(
        "IndexerFeed",
        Commands.startEnd(
            () -> m_indexerSubsystem.index(),
            () -> m_indexerSubsystem.stopindexer(),
            m_indexerSubsystem));

    NamedCommands.registerCommand(
        "IndexerReverse",
        Commands.startEnd(
            () -> m_indexerSubsystem.reverse(),
            () -> m_indexerSubsystem.stopindexer(),
            m_indexerSubsystem));

    NamedCommands.registerCommand(
        "IndexerStop",
        Commands.runOnce(
            () -> m_indexerSubsystem.stopindexer(),
            m_indexerSubsystem));

    // Spin up flywheels + feed for 10 seconds
    NamedCommands.registerCommand(
        "shoot",
        Commands.run(() -> {
            ShotCalculator.ShootingParameters params;
 
            if (m_visionSubsystem.hasHubTarget()) {
                double dist = m_visionSubsystem.getDistanceToHub() - 1.2;
                Rotation2d driveAngle = m_visionSubsystem.getAllianceHubPosition()
                    .minus(m_robotDrive.getPose().getTranslation()).getAngle();
                params = ShotCalculator.getInstance().calculateFromDistance(dist, driveAngle);
            } else {
                // No vision target — use safe defaults
                m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMinAngleRad);
                m_shooterSubsystem.setFlywheelVelocities(
                    ShooterConstants.defaultFlywheelSpeedRPM,
                    ShooterConstants.defaultFlywheelSpeedRPM);
                if (m_shooterSubsystem.isReadyToShoot()) {
                    m_shooterSubsystem.feedNote();
                    m_indexerSubsystem.index();
                }
                return;
            }
 
            m_shooterSubsystem.setHoodPosition(params.hoodAngleRad());
            m_shooterSubsystem.setFlywheelVelocities(
                params.flywheelSpeedRPM(),
                params.flywheelSpeedRPM());
 
            if (m_shooterSubsystem.isReadyToShoot()) {
                m_shooterSubsystem.feedNote();
                m_indexerSubsystem.index();
            }
        }, m_shooterSubsystem, m_indexerSubsystem)
        .finallyDo(() -> {
            m_shooterSubsystem.setFlywheelVelocities(0, 0);
            m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMinAngleRad);
            m_shooterSubsystem.stopFeeder();
            m_indexerSubsystem.stopindexer();
            ShotCalculator.getInstance().clearParameters();
        }));

    // Same as above but with slapdown jitter while shooting
    NamedCommands.registerCommand(
        "JitterShoot10s",
        Commands.run(() -> {
            ShotCalculator.ShootingParameters params;
 
            if (m_visionSubsystem.hasHubTarget()) {
                double dist = m_visionSubsystem.getDistanceToHub() - 1.2;
                Rotation2d driveAngle = m_visionSubsystem.getAllianceHubPosition()
                    .minus(m_robotDrive.getPose().getTranslation()).getAngle();
                params = ShotCalculator.getInstance().calculateFromDistance(dist, driveAngle);
            } else {
                m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMinAngleRad);
                m_shooterSubsystem.setFlywheelVelocities(
                    ShooterConstants.defaultFlywheelSpeedRPM,
                    ShooterConstants.defaultFlywheelSpeedRPM);
                if (m_shooterSubsystem.isReadyToShoot()) {
                    m_shooterSubsystem.feedNote();
                    m_indexerSubsystem.index();
                    if (m_intakeSubsystem.count == 0) m_intakeSubsystem.slapdowndown();
                    else m_intakeSubsystem.slapdownup();
                }
                return;
            }
 
            m_shooterSubsystem.setHoodPosition(params.hoodAngleRad());
            m_shooterSubsystem.setFlywheelVelocities(
                params.flywheelSpeedRPM(),
                params.flywheelSpeedRPM());
 
            if (m_shooterSubsystem.isReadyToShoot()) {
                m_shooterSubsystem.feedNote();
                m_indexerSubsystem.index();
                if (m_intakeSubsystem.count == 0) m_intakeSubsystem.slapdowndown();
                else m_intakeSubsystem.slapdownup();
            }
        }, m_shooterSubsystem, m_indexerSubsystem, m_intakeSubsystem)
        .finallyDo(() -> {
            m_shooterSubsystem.setFlywheelVelocities(0, 0);
            m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMinAngleRad);
            m_shooterSubsystem.stopFeeder();
            m_indexerSubsystem.stopindexer();
            m_intakeSubsystem.slapdownup();
            ShotCalculator.getInstance().clearParameters();
        })
        .withTimeout(10.0));
 

    // Stop everything and retract intake
    NamedCommands.registerCommand(
        "StopIntakeAndShooter",
        Commands.runOnce(
            () -> {
              m_intakeSubsystem.slapdownup();
              m_intakeSubsystem.stopintake();
              m_indexerSubsystem.stopindexer();
              m_shooterSubsystem.stopFeeder();
              m_shooterSubsystem.setFlywheelVelocities(0, 0);
            },
            m_intakeSubsystem, m_indexerSubsystem, m_shooterSubsystem)
            .ignoringDisable(true));

    NamedCommands.registerCommand(
        "StopIntake",
        Commands.runOnce(
            () -> {
              m_intakeSubsystem.stopintake();
            },
            m_intakeSubsystem)
            .ignoringDisable(true));
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    /**
     * DRIVER CONTROLS (Port 0)
     * - Left Joystick ........ Drive
     * - Right Joystick ....... Turn
     * - B Button ............. Toggle Field Relative
     * - LT (Left Trigger) .... Reset Gyro to 0
     * - A Button ............. Snap to 0°
     * - X .................... Lock Wheels to X
     * - POV Up ............... Test Everything
     * - POV Down ............. Toggle Offense/Defense Speed
     * ----------------------------------------------
     * OPERATOR CONTROLS (Port 1)
     * - RT (Right Trigger) ... Shooting
     * - LT (Left Trigger) .... Passing (cross-court)
     * - RB (Right Bumper) .... Intake
     * - LB (Left Bumper) ..... Outtake
     * - X Button ............. Unjam
     * - Y Button ............. Feed Note
     * - B Button ............. Indexer
     * - POV Down ............. Slapdown toggle
     * - POV Up ............... Pass (neutral to alliance)
     * - POV Left ............. Hood Angle Down
     * - POV Right ............ Hood Angle Up
     */

    // X: Lock wheels to X formation
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .onTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive)
            .until(() ->
                Math.abs(m_driverController.getLeftY()) > OIConstants.kDriveDeadband ||
                Math.abs(m_driverController.getLeftX()) > OIConstants.kDriveDeadband ||
                Math.abs(m_driverController.getRightX()) > OIConstants.kDriveDeadband
            ));

    // A: Snap to 0°
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .whileTrue(new RunCommand(() -> {
          double heading = m_robotDrive.getHeading();
          if (Math.abs(heading) < DriveConstants.snapTolerance) {
            m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                0,
                m_fieldRelative);
          } else {
            double snapOutput = m_snapController.calculate(heading, 0);
            m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                snapOutput,
                m_fieldRelative);
          }
        }, m_robotDrive));

    // B: Toggle field relative
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .onTrue(new InstantCommand(() -> {
          m_fieldRelative = !m_fieldRelative;
          System.out.println("Driving field relative is " + m_fieldRelative);
        }));

    // POV Up: System test
    new POVButton(m_driverController, 0)
        .onTrue(new SystemTestCommand(m_robotDrive));

    // POV Down: Toggle offense/defense speed
    new POVButton(m_driverController, 180)
        .onTrue(new InstantCommand(() -> {
          if (speed_Offense_Defense == offense_speed) {
            speed_Offense_Defense = defense_speed;
          } else {
            speed_Offense_Defense = offense_speed;
          }
          System.out.println("Speed multiplier: " + speed_Offense_Defense);
        }));

    // Y: System test (duplicate of POV Up — kept from original)
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new SystemTestCommand(m_robotDrive));

    // LT: Zero gyro
    new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.5)
        .whileTrue(new RunCommand(() -> {
          System.out.println("Zeroing gyro!");
          m_robotDrive.zeroHeading();
        }, m_robotDrive));

    // OPERATOR ------------------------------------------

    // RB: Intake
    new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value)
        .whileTrue(Commands.startEnd(
            () -> m_intakeSubsystem.intake(),
            () -> m_intakeSubsystem.stopintake()));

    // LB: Outtake
    new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
        .whileTrue(Commands.startEnd(
            () -> m_intakeSubsystem.outtake(),
            () -> m_intakeSubsystem.stopintake()));

    // Y: Feed note
    new JoystickButton(m_operatorController, XboxController.Button.kY.value)
        .whileTrue(Commands.startEnd(
            () -> m_shooterSubsystem.feedNote(),
            () -> m_shooterSubsystem.stopFeeder()));

    // B: Run indexer
    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
        .whileTrue(Commands.startEnd(
            () -> m_indexerSubsystem.setVoltage(6),
            () -> m_indexerSubsystem.stopindexer()));

    // POV Down: Slapdown toggle
    new POVButton(m_operatorController, 180)
        .onTrue(Commands.runOnce(() -> {
          if (m_intakeSubsystem.count == 0) {
            m_intakeSubsystem.slapdowndown();
          } else {
            m_intakeSubsystem.slapdownup();
          }
        }, m_intakeSubsystem));

    // POV Up: Pass (neutral zone to alliance zone)
    new POVButton(m_operatorController, 0)
        .whileTrue(Commands.startEnd(
            () -> {
              m_shooterSubsystem.setFlywheelVelocities(
                  NetworkValues.getInstance().getFlywheelRPM() - 1000,
                  NetworkValues.getInstance().getFlywheelRPM() - 1000);
              m_shooterSubsystem.setHoodPosition(Units.degreesToRadians(20.0));
              m_shooterSubsystem.feedNote();
              m_indexerSubsystem.index();
            },
            () -> {
              m_shooterSubsystem.setFlywheelVelocities(0, 0);
              m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMinAngleRad);
              m_shooterSubsystem.stopFeeder();
              m_indexerSubsystem.stopindexer();
            },
            m_shooterSubsystem, m_indexerSubsystem));

    // LT: Cross-court pass
    new Trigger(() -> m_operatorController.getLeftTriggerAxis() > 0.5)
        .whileTrue(Commands.startEnd(
            () -> {
              m_shooterSubsystem.setFlywheelVelocities(
                  NetworkValues.getInstance().getFlywheelRPM(),
                  NetworkValues.getInstance().getFlywheelRPM());
              m_shooterSubsystem.setHoodPosition(Units.degreesToRadians(30.0));
              m_shooterSubsystem.feedNote();
              m_indexerSubsystem.index();
            },
            () -> {
              m_shooterSubsystem.setFlywheelVelocities(0, 0);
              m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMinAngleRad);
              m_shooterSubsystem.stopFeeder();
              m_indexerSubsystem.stopindexer();
            },
            m_shooterSubsystem, m_indexerSubsystem));

    // RT: Shoot with shot calculator
    new Trigger(() -> m_operatorController.getRightTriggerAxis() > 0.5)
        .whileTrue(Commands.run(() -> {
          ShotCalculator.ShootingParameters params;

          System.out.println("Using vision: " + m_visionSubsystem.hasHubTarget());
          if (m_visionSubsystem.hasHubTarget()) {
            double dist = m_visionSubsystem.getDistanceToHub() - 1.2;
            System.out.println("Distance to hub: " + dist);
            Rotation2d driveAngle = m_visionSubsystem.getAllianceHubPosition()
                .minus(m_robotDrive.getPose().getTranslation()).getAngle();
            params = ShotCalculator.getInstance().calculateFromDistance(dist, driveAngle);
          } else {
            m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMinAngleRad);
            m_shooterSubsystem.setFlywheelVelocities(
                ShooterConstants.defaultFlywheelSpeedRPM,
                ShooterConstants.defaultFlywheelSpeedRPM);
            if (m_shooterSubsystem.isReadyToShoot()) {
              m_shooterSubsystem.feedNote();
              m_indexerSubsystem.index();
            }
            return;
          }

          System.out.println("Delta hood angle: " + (Units.radiansToDegrees(params.hoodAngleRad())
              - Units.radiansToDegrees(m_shooterSubsystem.getHoodPosition())));
          System.out.println("Delta flywheel RPM: " + (params.flywheelSpeedRPM()
              - m_shooterSubsystem.getAverageFlywheelVelocity()));

          m_shooterSubsystem.setHoodPosition(params.hoodAngleRad());
          m_shooterSubsystem.setFlywheelVelocities(params.flywheelSpeedRPM(), params.flywheelSpeedRPM());

          if (m_shooterSubsystem.isReadyToShoot()) {
            m_shooterSubsystem.feedNote();
            m_indexerSubsystem.index();
          }
        }, m_shooterSubsystem, m_indexerSubsystem)
            .finallyDo(() -> {
              m_shooterSubsystem.setFlywheelVelocities(0, 0);
              m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMinAngleRad);
              m_shooterSubsystem.stopFeeder();
              m_indexerSubsystem.stopindexer();
              ShotCalculator.getInstance().clearParameters();
            }));

    // X: Unjam / eject note
    new JoystickButton(m_operatorController, XboxController.Button.kX.value)
        .whileTrue(Commands.startEnd(
            () -> {
              m_shooterSubsystem.ejectNote();
              m_shooterSubsystem.setFlywheelVelocities(
                  -NetworkValues.getInstance().getDefaultFlywheelRPM(),
                  -NetworkValues.getInstance().getDefaultFlywheelRPM());
              m_indexerSubsystem.reverse();
            },
            () -> {
              m_shooterSubsystem.stopFeeder();
              m_shooterSubsystem.setFlywheelVelocities(0, 0);
              m_indexerSubsystem.stopindexer();
            },
            m_shooterSubsystem, m_indexerSubsystem));

    // POV Right: Hood up
    new POVButton(m_operatorController, 90)
        .onTrue(Commands.runOnce(() -> {
          m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMaxAngleRad);
          System.out.println("Hood angle: " + Units.radiansToDegrees(m_shooterSubsystem.getHoodPosition()));
        }, m_shooterSubsystem));

    // POV Left: Hood down
    new POVButton(m_operatorController, 270)
        .onTrue(Commands.runOnce(() -> {
          m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMinAngleRad);
          System.out.println("Hood angle: " + Units.radiansToDegrees(m_shooterSubsystem.getHoodPosition()));
        }, m_shooterSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * FIX: Now correctly returns the selected PathPlanner auto from autoChooser.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}