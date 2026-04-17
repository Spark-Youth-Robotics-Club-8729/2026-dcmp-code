// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.indexerconstants;
import frc.robot.Constants.intakeconstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.NetworkValues;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import java.util.Timer;
import frc.robot.command.SystemTestCommand;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
    }
  public ShooterSubsystem getShooterSubsystem() {
    return m_shooterSubsystem;
  }
  public IntakeSubsystem getIntakeSubsystem() {
    return m_intakeSubsystem;
  }
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  private final PIDController m_snapController = new PIDController(DriveConstants.kPSnap, DriveConstants.kISnap, DriveConstants.kDSnap);
  
  //initialize sppeds and relative
  boolean m_fieldRelative = true;
  boolean m_indexer = true;
  boolean m_slapdown = true;
  boolean m_jitterToggle = true;
  

  private final double offense_speed = 1.0;
  private final double defense_speed = 1.7;  // 30% times faster than offense

   private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
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
    m_snapController.enableContinuousInput(-180.0, 180.0);
    m_snapController.setTolerance(DriveConstants.snapTolerance);
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband) 
                * NetworkValues.getInstance().GetMaxSpeedInMeters(), // Added this
            -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband) 
                * NetworkValues.getInstance().GetMaxSpeedInMeters(), // Added this
            -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)
                * NetworkValues.getInstance().GetMaxTurnSpeed(),
            m_fieldRelative),
        m_robotDrive)); 

        NetworkValues.getInstance().systemTest(
        new SystemTestCommand(m_robotDrive).withName("Full System Test"));
  }

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
                // No vision target — use safe defaults
                m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMinAngleRad);
                m_shooterSubsystem.setFlywheelVelocities(
                    ShooterConstants.defaultFlywheelSpeedRPM+500,
                    ShooterConstants.defaultFlywheelSpeedRPM+500);
                if (m_shooterSubsystem.isReadyToShoot()) {
                    m_shooterSubsystem.feedNote();
                    m_indexerSubsystem.index();
                }
                return;
        }, m_shooterSubsystem, m_indexerSubsystem)
        .finallyDo(() -> {
            m_shooterSubsystem.setFlywheelVelocities(0, 0);
            m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMinAngleRad);
            m_shooterSubsystem.stopFeeder();
            m_indexerSubsystem.stopindexer();
        }));

    // Same as above but with slapdown jitter while shooting.. in the works
    /*
    NamedCommands.registerCommand(
        "JitterShoot10s",
        Commands.run(() -> {
 
                m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMinAngleRad);
                m_shooterSubsystem.setFlywheelVelocities(
                    ShooterConstants.defaultFlywheelSpeedRPM,
                    ShooterConstants.defaultFlywheelSpeedRPM);
                if (m_shooterSubsystem.isReadyToShoot()) {
                    m_shooterSubsystem.feedNote();
                    m_indexerSubsystem.index();
                    
                    // Apply jitter to slapdown during shooting
                    double currentAngle = m_intakeSubsystem.currentPosition();
                    if (m_jitterToggle) {
                        System.out.println("bringing jitter UP");
                        m_intakeSubsystem.slapdownjitterUp(currentAngle);
                        m_jitterToggle = false;
                    } else {
                        System.out.println("bringing jitter DOWN");
                        m_intakeSubsystem.slapdownjitterDown(currentAngle);
                        m_jitterToggle = true;
                    }
                }
                return;
        }, m_shooterSubsystem, m_indexerSubsystem, m_intakeSubsystem)
        .finallyDo(() -> {
            m_shooterSubsystem.setFlywheelVelocities(0, 0);
            m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMinAngleRad);
            m_shooterSubsystem.stopFeeder();
            m_indexerSubsystem.stopindexer();
            m_intakeSubsystem.slapdownup();
        })
        .withTimeout(10.0));
        */
 

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
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
    private void configureButtonBindings() {
        /**
         * DRIVER CONTROLS (Port 0)
         * - Left Joystick ........ Drive (works!)
         * - Right Joystick ....... Turn (works!)
         * - B Button ............. Robot to Field (works)
         * - LT (Left Trigger) .... Reset Gyro to 0
         * - A Button ............. Snap to 0° (works!)
         * - X ... Lock Wheels to X (works!)
         * - POV Up ............... Test Everything (works!)
         * ----------------------------------------------
         * OPERATOR CONTROLS (Port 1)
         * A ................. tower shot, waits 0.3 sec
     * B (Hold) ............ Passing Shot: alliance to netural, waits 0.3s, then feeds
     * X (Hold) ............ Unjam/Eject: Reverses shooter, feeder, and indexer
     * Y (Hold) ............ Flywheel Only: Spins flywheels
     * Right Bumper (RB) ... Intake Toggle: Toggles intake motor
     * Left Bumper (LB) .... Outtake (Hold): Runs intake in reverse
     * Right Trigger (>0.5). Hub Shot: Sets Hood/RPM for Hub, waits 0.3s, then feeds
     * Left Trigger (>0.5) . Court Pass: Cross-court pass logic (Court Hood/RPM), waits 0.3 sec
     * POV Up (0) .......... Manual Index: Manually runs the indexer at feed voltage
     * POV Down (180) ...... Slapdown Toggle: Toggles the intake slapdown position
     * POV Right (90) ...... Adding the Jitter
     * POV Left (270) ...... Hood MIN: Sets hood to min angle
         */

        // Single press of X sets the robot into X formation
        new JoystickButton(m_driverController, XboxController.Button.kX.value)
            .onTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive)
                .until(() ->
                    Math.abs(m_driverController.getLeftY()) > OIConstants.kDriveDeadband ||
                    Math.abs(m_driverController.getLeftX()) > OIConstants.kDriveDeadband ||
                    Math.abs(m_driverController.getRightX()) > OIConstants.kDriveDeadband
                ));

        // Snaps to 0 of the gyro
        new JoystickButton(m_driverController, XboxController.Button.kA.value)
         .whileTrue(new RunCommand(() -> {
                double heading = m_robotDrive.getHeading();
                //System.out.println("Heading: " + heading);   // we love debug prints
                if (Math.abs(heading) < DriveConstants.snapTolerance) {  // if heading within tolerance, it wont move
                    double snapOutput = 0;
                    m_robotDrive.drive(
                        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                        snapOutput,
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

        //switch from field to robot
        new JoystickButton(m_driverController, XboxController.Button.kB.value)
            .onTrue(new InstantCommand(() ->
        {
             m_fieldRelative = !m_fieldRelative;
             System.out.println("Driving field relative is "+m_fieldRelative);
        }
             ));

             // system test command
        new POVButton(m_driverController, 0)   
            .onTrue(new SystemTestCommand(m_robotDrive));

        // SHOULD 0 the gyro...
        new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.5)
           .whileTrue(new RunCommand(() -> {
                m_robotDrive.resetGyro();
                System.out.println("Zeroing gyro!");
            }, m_robotDrive));


        // OPERATOR          
        
        //intake toggle right bumper
        new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value)
            .onTrue(Commands.runOnce(()->{
                    if(m_indexer){
                        m_intakeSubsystem.intake();
                        m_indexer = false;
                    }
                    else {
                        m_intakeSubsystem.stopintake();
                        m_indexer = true;
                    }
                },m_intakeSubsystem));

                // outtake
        new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
           .whileTrue(Commands.startEnd(() -> m_intakeSubsystem.outtake(), ()->m_intakeSubsystem.stopintake()));

           // run flywheels at flywheel rpm
        new JoystickButton(m_operatorController, XboxController.Button.kY.value)
           .whileTrue(Commands.startEnd(() ->  m_shooterSubsystem.setFlywheelVelocities(NetworkValues.getInstance().getFlywheelRPM(), NetworkValues.getInstance().getFlywheelRPM()), ()->m_shooterSubsystem.setFlywheelVelocities(0, 0)));

           // passing from alliance to neutral
        new JoystickButton(m_operatorController, XboxController.Button.kB.value)
        .whileTrue(
            Commands.sequence(
                Commands.runOnce(() -> {
                    double hoodAngleRad = Units.degreesToRadians(
                        NetworkValues.getInstance().getPassHoodAngle()
                    );
                    m_shooterSubsystem.setFlywheelVelocities(
                        NetworkValues.getInstance().getPassRPM(),
                        NetworkValues.getInstance().getPassRPM()
                    );
                    m_shooterSubsystem.setHoodPosition(hoodAngleRad);
                }, m_shooterSubsystem),

                // Wait for spin-up
                Commands.waitSeconds(0.3),

                // Feed note
                Commands.run(() -> {
                    m_shooterSubsystem.feedNote();
                    m_indexerSubsystem.index();
                }, m_shooterSubsystem, m_indexerSubsystem)
            )
            .finallyDo(() -> {
                // Stop everything when button released
                m_shooterSubsystem.setFlywheelVelocities(0, 0);
                m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMinAngleRad);
                m_shooterSubsystem.stopFeeder();
                m_indexerSubsystem.stopindexer();
            })
        );


           // toggle slapdown (add a check to set m_slapdown based on current angle)
        new POVButton(m_operatorController, 180)
            .onTrue(Commands.runOnce(()->{m_intakeSubsystem.slapdowntoggle();}));
        
            // jitter....
        new POVButton(m_operatorController, 90)
           .whileTrue(Commands.runOnce(()->{m_intakeSubsystem.jittertoggle();}));
 
            // set hood anlge to min
        new POVButton(m_operatorController, 270)
            .onTrue(Commands.runOnce(()->{
                m_shooterSubsystem.setHoodPosition(Constants.ShooterConstants.hoodMinAngleRad);;
            },m_shooterSubsystem));
        
        // run indexer
        new POVButton(m_operatorController, 0)
        .whileTrue(Commands.startEnd(() -> m_indexerSubsystem.setVoltage(indexerconstants.feedVolts), ()->m_indexerSubsystem.stopindexer()));
        
        // A: Tower Shot — 20 deg hood, 3200 RPM (spin up then feed)
        new JoystickButton(m_operatorController, XboxController.Button.kA.value)
        .whileTrue(
            Commands.sequence(
            Commands.runOnce(() -> {
                // network tables
                double rpm = NetworkValues.getInstance().getTowerRPM();
                double hoodDeg = NetworkValues.getInstance().getTowerHoodAngle();
                
                m_shooterSubsystem.setFlywheelVelocities(rpm, rpm);
                m_shooterSubsystem.setHoodPosition(Units.degreesToRadians(hoodDeg));
            }, m_shooterSubsystem),
            Commands.waitSeconds(0.3),
            Commands.run(() -> {
                m_shooterSubsystem.feedNote();
                m_indexerSubsystem.index();
            }, m_shooterSubsystem, m_indexerSubsystem)
        )
        .finallyDo(() -> {
            m_shooterSubsystem.setFlywheelVelocities(0, 0);
            m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMinAngleRad);
            m_shooterSubsystem.stopFeeder();
            m_indexerSubsystem.stopindexer();
        })
        );

        //cross court passing from alliance zone to alliance zone (using same speed as netural to alliance but different hood)
        new Trigger(() -> m_operatorController.getLeftTriggerAxis() > 0.5)
        .whileTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
                double rpm = NetworkValues.getInstance().getCourtRPM();
                double hoodDeg = NetworkValues.getInstance().getCourtHoodAngle();
                
                m_shooterSubsystem.setFlywheelVelocities(rpm, rpm);
                m_shooterSubsystem.setHoodPosition(Units.degreesToRadians(hoodDeg));
            }, m_shooterSubsystem),
            Commands.waitSeconds(0.3),
            Commands.run(() -> {
                m_shooterSubsystem.feedNote();
                m_indexerSubsystem.index();
            }, m_shooterSubsystem, m_indexerSubsystem)
            )
        .finallyDo(() -> {
            m_shooterSubsystem.setFlywheelVelocities(0, 0);
            m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMinAngleRad);
            m_shooterSubsystem.stopFeeder();
            m_indexerSubsystem.stopindexer();
        })
        );

        // RT: Hub Shot — 10 deg hood, 2500 RPM (spin up then feed)
        new Trigger(() -> m_operatorController.getRightTriggerAxis() > 0.5)
        .whileTrue(
            Commands.sequence(
                Commands.runOnce(() -> {
                    double rpm = NetworkValues.getInstance().getHubRPM();
                    double hoodDeg = NetworkValues.getInstance().getHubHoodAngle();
                    
                    m_shooterSubsystem.setFlywheelVelocities(rpm, rpm);
                    m_shooterSubsystem.setHoodPosition(Units.degreesToRadians(hoodDeg));
                }, m_shooterSubsystem),
                Commands.waitSeconds(0.3),
                Commands.run(() -> {
                    m_shooterSubsystem.feedNote();
                    m_indexerSubsystem.index();
                }, m_shooterSubsystem, m_indexerSubsystem)
                )
            .finallyDo(() -> {
                m_shooterSubsystem.setFlywheelVelocities(0, 0);
                m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMinAngleRad);
                m_shooterSubsystem.stopFeeder();
                m_indexerSubsystem.stopindexer();
            })
            );

            //reverse everything
        new JoystickButton(m_operatorController, XboxController.Button.kX.value)
            .whileTrue(Commands.startEnd(
                () -> {
                    m_shooterSubsystem.ejectNote();
                    m_shooterSubsystem.setFlywheelVelocities(-NetworkValues.getInstance().getDefaultFlywheelRPM(), -NetworkValues.getInstance().getDefaultFlywheelRPM());
                    m_indexerSubsystem.reverse();
                },
                () -> {
                    m_shooterSubsystem.stopFeeder();
                    m_shooterSubsystem.setFlywheelVelocities(0, 0);
                    m_indexerSubsystem.stopindexer();
                },
                m_shooterSubsystem, m_indexerSubsystem
            ));
            
        // //bring hood up (add jitter one later...)
        // new POVButton(m_operatorController, 90)
        //      .onTrue(Commands.runOnce(()->{
        //          m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMaxAngleRad);
        //          System.out.println("Hood angle: " + Units.radiansToDegrees(m_shooterSubsystem.getHoodPosition()));
        //      }, m_shooterSubsystem));

        //      // bring hood down
        // new POVButton(m_operatorController, 270)
        //      .onTrue(Commands.runOnce(()->{
        //          m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMinAngleRad);
        //          System.out.println("Hood angle: " + Units.radiansToDegrees(m_shooterSubsystem.getHoodPosition()));
        //      }, m_shooterSubsystem)); 



    }
        




  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    System.out.println("here");
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, true));   // auto needs to be in field relative i think? if not, then just change the true to false
  }

  
} 