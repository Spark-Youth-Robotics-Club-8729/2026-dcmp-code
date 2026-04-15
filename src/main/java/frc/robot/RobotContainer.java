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
  

  private final double offense_speed = 1.0;
  private final double defense_speed = 1.7;  // 30% times faster than offense

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                m_fieldRelative),
            m_robotDrive));
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
         * - LT (Left Trigger) .... Reset Gyro to 0 (works!)
         * - A Button ............. Snap to 0° (works!)
         * - X ... Lock Wheels to X (works!)
         * - POV Down ............. Swtich to defnese speed (DOES NOT WORKKK)
         * - POV Up ............... Test Everything (works!)
         * ----------------------------------------------
         * OPERATOR CONTROLS (Port 1)
         * - RT (Right Trigger) ... Shooting (havent tested yet)
         * - LT (Left Trigger) .... Passing (havent tested yet)
         * - RB (Right Bumper) .... Intake (havent tested yet)
         * - LB (Left Bumper) ..... Outtake (havent tested yet)
         * - X Button ............. Unjam (havent tested yet)
         * - POV Up ............... Tunable shooting with elastic 
         * - POV Down ............. Slapdown (havent tested yet)
         * - POV Left ............. Hood Angle Down (havent tested yet)
         * - POV Right ............ Hood Angle Up (havent tested yet)
         * - Y .................... flywheel ONLY
         * - A .................... manual shoot
         */

        // Single press of X sets the robot into X formation
        new JoystickButton(m_driverController, XboxController.Button.kX.value)
            .onTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive)
                .until(() ->
                    Math.abs(m_driverController.getLeftY()) > OIConstants.kDriveDeadband ||
                    Math.abs(m_driverController.getLeftX()) > OIConstants.kDriveDeadband ||
                    Math.abs(m_driverController.getRightX()) > OIConstants.kDriveDeadband
                ));

        // Single press of Start button zeroes the gyro heading
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


        // In configureButtonBindings()

        new JoystickButton(m_driverController, XboxController.Button.kB.value)
            .onTrue(new InstantCommand(() ->
        {
             m_fieldRelative = !m_fieldRelative;
             System.out.println("Driving field relative is "+m_fieldRelative);
        }
             ));

        new POVButton(m_driverController, 0)   
            .onTrue(new SystemTestCommand(m_robotDrive));


        /* 
        new POVButton(m_driverController, 180) 
            .onTrue(new InstantCommand(() ->
        {
            DriveConstants.kMaxSpeedMetersPerSecond = 0.1;
        }
             ));

        */

        new JoystickButton(m_driverController, XboxController.Button.kY.value)    
            .onTrue(new SystemTestCommand(m_robotDrive));

        new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.5)
           .whileTrue(new RunCommand(() -> {
                System.out.println("Zeroing gyro!");
                m_robotDrive.zeroHeading();
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

        new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
           .whileTrue(Commands.startEnd(() -> m_intakeSubsystem.outtake(), ()->m_intakeSubsystem.stopintake()));

        new JoystickButton(m_operatorController, XboxController.Button.kY.value)
           .whileTrue(Commands.startEnd(() ->  m_shooterSubsystem.setFlywheelVelocities(NetworkValues.getInstance().getFlywheelRPM(), NetworkValues.getInstance().getFlywheelRPM()), ()->m_shooterSubsystem.setFlywheelVelocities(0, 0)));

        new JoystickButton(m_operatorController, XboxController.Button.kB.value)
           .whileTrue(Commands.startEnd(() -> m_indexerSubsystem.setVoltage(indexerconstants.feedVolts), ()->m_indexerSubsystem.stopindexer()));


           // toggle slapdown (add a check to set m_slapdown based on current angle... if angle = 0 vs. greater than 1 rad)
        new POVButton(m_operatorController, 180)
            .onTrue(Commands.runOnce(()->{
                if(m_slapdown) {
                    m_intakeSubsystem.slapdowndown();
                    m_slapdown = false;
                    //System.out.println("angle"+slapdownencoder.getPosition());
                }
                else {
                    m_intakeSubsystem.slapdownup();
                    m_slapdown = true;
                }
            },m_intakeSubsystem));
        

        // new POVButton(m_operatorController, 90)
        //     .onTrue(Commands.runOnce(()->{
        //         m_intakeSubsystem.slapdowndown();
        //     },m_intakeSubsystem));

    //down arrow does up and down, right arrow does slapdown DOWN and left arrow does slapdown UP
        // new POVButton(m_operatorController, 270)
        //     .onTrue(Commands.runOnce(()->{
        //         m_intakeSubsystem.slapdownup();
        //     },m_intakeSubsystem));
        
        // passing from neutral zone to alliance zone (LIVE EDITABLE HOOD AND SPEEDS) with time delay for indexer
        new POVButton(m_operatorController, 0)
        .whileTrue(
            Commands.sequence(
                Commands.runOnce(() -> {
                    double hoodAngleRad = Units.degreesToRadians(
                        NetworkValues.getInstance().getPassingHoodAngle()
                    );
                    m_shooterSubsystem.setFlywheelVelocities(
                        NetworkValues.getInstance().getFlywheelRPM(),
                        NetworkValues.getInstance().getFlywheelRPM()
                    );
                    m_shooterSubsystem.setHoodPosition(hoodAngleRad);
                }, m_shooterSubsystem),

                // Wait for spin-up
                Commands.waitSeconds(0.5),

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
        // A: Tower Shot — 20 deg hood, 3200 RPM (spin up then feed)
        new JoystickButton(m_operatorController, XboxController.Button.kA.value)
        .whileTrue(
            Commands.sequence(
                Commands.runOnce(() -> {
                    m_shooterSubsystem.setFlywheelVelocities(ShooterConstants.towerFlywheelRPM, ShooterConstants.towerFlywheelRPM);
                    m_shooterSubsystem.setHoodPosition(Units.degreesToRadians(ShooterConstants.towerHoodAngleDeg));
                }, m_shooterSubsystem),
                Commands.waitSeconds(0.5),
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
        new Trigger(()->m_operatorController.getLeftTriggerAxis()>0.5)
            .whileTrue(Commands.startEnd(() -> {
                m_shooterSubsystem.setFlywheelVelocities(NetworkValues.getInstance().getFlywheelRPM(), NetworkValues.getInstance().getFlywheelRPM());
                m_shooterSubsystem.setHoodPosition(Units.degreesToRadians(30.0));    //maybe lower this
                m_shooterSubsystem.feedNote();
                m_indexerSubsystem.index();
            }, 
            ()->{
                m_shooterSubsystem.setFlywheelVelocities(0, 0);
                m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMinAngleRad);
                m_shooterSubsystem.stopFeeder();
                m_indexerSubsystem.stopindexer();
            },m_shooterSubsystem,m_indexerSubsystem));

        // RT: Hub Shot — 10 deg hood, 2500 RPM (spin up then feed)
        new Trigger(() -> m_operatorController.getRightTriggerAxis() > 0.5)
            .whileTrue(
                Commands.sequence(
                    Commands.runOnce(() -> {
                        m_shooterSubsystem.setFlywheelVelocities(ShooterConstants.hubFlywheelRPM, ShooterConstants.hubFlywheelRPM);
                        m_shooterSubsystem.setHoodPosition(Units.degreesToRadians(ShooterConstants.hubHoodAngleDeg));
                    }, m_shooterSubsystem),
                    Commands.waitSeconds(0.5),
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
            
        //bring hood up
        new POVButton(m_operatorController, 90)
             .onTrue(Commands.runOnce(()->{
                 m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMaxAngleRad);
                 System.out.println("Hood angle: " + Units.radiansToDegrees(m_shooterSubsystem.getHoodPosition()));
             }, m_shooterSubsystem));

             // bring hood down
        new POVButton(m_operatorController, 270)
             .onTrue(Commands.runOnce(()->{
                 m_shooterSubsystem.setHoodPosition(ShooterConstants.hoodMinAngleRad);
                 System.out.println("Hood angle: " + Units.radiansToDegrees(m_shooterSubsystem.getHoodPosition()));
             }, m_shooterSubsystem)); 



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