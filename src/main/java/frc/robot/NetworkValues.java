package frc.robot;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShotCalculator;
import edu.wpi.first.math.util.Units;

public class NetworkValues {
    private static NetworkValues instance;
    NetworkTableInstance inst;
    private final Field2d m_field;

    // OFFENSE / DEFENSE TOGGLE
    private BooleanSubscriber offenseModeSub;
    private boolean offenseMode = true;

    //PASSING HOOD ANGLE (LIVE TUNE)
    private DoublePublisher passHoodAnglePub;
    private DoubleSubscriber passHoodAngleSub;
    private double passHoodAngle = ShooterConstants.kDefaultPassHoodAngle;

    //live tune max speed
    DoublePublisher maxSpeedInMetersPub;
    DoubleSubscriber maxSpeedInMetersSub;
    double maxSpeedInMeters = DriveConstants.kMaxSpeedMetersPerSecond;

    // live tune max flywheel RPM
    DoublePublisher flywheelRPMPub;
    DoubleSubscriber flywheelRPMSub;
    double flywheelRPM = ShooterConstants.maxFlywheelSpeedRPM;

    //live tune normal flywheel RPM
    DoublePublisher defaultFlywheelRPMPub;
    DoubleSubscriber defaultFlywheelRPMSub;
    double defaultFlywheelRPM = ShooterConstants.defaultFlywheelSpeedRPM;

    //Show distance to hub
    DoublePublisher distanceToHubPub;
    // Show limelight (vision) measured distance to hub
    DoublePublisher visionDistanceToHubPub;

    // Shooter telemetry — published for Elastic Testing tab
    private DoublePublisher shooterHoodAnglePub;
    private DoublePublisher shooterFlywheelRPMPub;
    private BooleanPublisher shooterAtSpeedPub;
    private BooleanPublisher shooterHoodAtPositionPub;

    // Drive telemetry — published for Elastic Driver Station tab
    private DoublePublisher driveSpeedMPSPub;

    public static NetworkValues getInstance(){
        if (instance == null) {
            instance = new NetworkValues();
        }
        return instance;
    }

    public NetworkValues()
    {
        inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable(("datatable"));

        DoubleTopic maxSpeedInMetersTopic = table.getDoubleTopic(("maxSpeedInMeters"));
        maxSpeedInMetersPub = maxSpeedInMetersTopic.publish();
        maxSpeedInMetersPub.set(maxSpeedInMeters);
        maxSpeedInMetersSub = maxSpeedInMetersTopic.subscribe(maxSpeedInMeters);

        DoubleTopic flywheelTopic = table.getDoubleTopic("passing RPM");
        flywheelRPMPub = flywheelTopic.publish();
        flywheelRPMPub.set(flywheelRPM);
        flywheelRPMSub = flywheelTopic.subscribe(flywheelRPM);

        DoubleTopic defaultFlywheelTopic = table.getDoubleTopic("defaultFlywheelRPM");
        defaultFlywheelRPMPub = defaultFlywheelTopic.publish();
        defaultFlywheelRPMPub.set(defaultFlywheelRPM);
        defaultFlywheelRPMSub = defaultFlywheelTopic.subscribe(defaultFlywheelRPM);

        // passing hood angle
        DoubleTopic hoodAngleTopic = table.getDoubleTopic("passHoodAngleDeg");
        passHoodAnglePub = hoodAngleTopic.publish();
        passHoodAnglePub.set(passHoodAngle);
        passHoodAngleSub = hoodAngleTopic.subscribe(passHoodAngle);

        //offense/defense
        offenseModeSub = table.getBooleanTopic("offenseMode").subscribe(true);

        // pose-based distance (from ShotCalculator / pose estimation)
        DoubleTopic distanceToHubPoseTopic = table.getDoubleTopic("distanceToHubPose");
        distanceToHubPub = distanceToHubPoseTopic.publish();

        // vision-based distance (from Limelight)
        DoubleTopic distanceToHubVisionTopic = table.getDoubleTopic("distanceToHubVision");
        visionDistanceToHubPub = distanceToHubVisionTopic.publish();

        // Shooter telemetry (for Elastic Testing tab)
        NetworkTable shooterTable = inst.getTable("shooter");
        shooterHoodAnglePub  = shooterTable.getDoubleTopic("hoodAngleDeg").publish();
        shooterFlywheelRPMPub = shooterTable.getDoubleTopic("flywheelRPM").publish();
        shooterAtSpeedPub    = shooterTable.getBooleanTopic("flywheelAtSpeed").publish();
        shooterHoodAtPositionPub = shooterTable.getBooleanTopic("hoodAtPosition").publish();

        // Drive telemetry (for Elastic Driver Station tab)
        NetworkTable driveTable = inst.getTable("drive");
        driveSpeedMPSPub = driveTable.getDoubleTopic("speedMPS").publish();

        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);
    }

    public double GetMaxSpeedInMeters() {
        if (offenseMode) {
            return maxSpeedInMeters; // Uses the live-tuned max speed
        } else {
            return 1.5; // Fixed "Defense" speed
        }
    }

    public double getFlywheelRPM() {
        return flywheelRPM;
    }

    public double getDefaultFlywheelRPM() {
        return defaultFlywheelRPM;
    }


    public boolean isOffenseMode() {
        return offenseMode;
    }

    public double getPassingHoodAngle() {
        return passHoodAngle;
    }

    public void periodic(RobotContainer robotContainer) {
        defaultFlywheelRPM = defaultFlywheelRPMSub.get();
        flywheelRPM = flywheelRPMSub.get();
        maxSpeedInMeters = maxSpeedInMetersSub.get();
        passHoodAngle = passHoodAngleSub.get();
        offenseMode = offenseModeSub.get();

        // publish pose-based distance calculated by ShotCalculator
        distanceToHubPub.set(ShotCalculator.getInstance().getDistanceToTarget());

        // publish vision-based limelight distance (may be NaN if no tag visible)
        double visionDist = robotContainer.getVisionSubsystem().getDistanceToHub();
        visionDistanceToHubPub.set(Double.isNaN(visionDist) ? -1.0 : visionDist);

        // Publish shooter telemetry for Elastic Testing tab
        var shooter = robotContainer.getShooterSubsystem();
        shooterHoodAnglePub.set(Units.radiansToDegrees(shooter.getHoodPosition()));
        shooterFlywheelRPMPub.set(shooter.getAverageFlywheelVelocity());
        shooterAtSpeedPub.set(shooter.areFlywheelsAtSpeed());
        shooterHoodAtPositionPub.set(shooter.isHoodAtPosition());

        // Publish drive telemetry for Elastic Driver Station tab
        driveSpeedMPSPub.set(GetMaxSpeedInMeters());

        m_field.setRobotPose(robotContainer.getDriveSubsystem().getPose());
    }

    public boolean isHubActive() {
        var allianceOpt = DriverStation.getAlliance();

        if (allianceOpt.isEmpty()) return false;

        Alliance alliance = allianceOpt.get();

        if (DriverStation.isAutonomousEnabled()) return true;
        if (!DriverStation.isTeleopEnabled()) return false;

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.isEmpty()) return true;

        boolean redInactiveFirst = gameData.charAt(0) == 'R';

        boolean shift1Active =
            (alliance == Alliance.Red) ? !redInactiveFirst : redInactiveFirst;

        if (matchTime > 130) return true;
        else if (matchTime > 105) return shift1Active;
        else if (matchTime > 80) return !shift1Active;
        else if (matchTime > 55) return shift1Active;
        else if (matchTime > 30) return !shift1Active;
        else return true;
    }

    public void close()
    {
        maxSpeedInMetersSub.close();
        flywheelRPMSub.close();
        defaultFlywheelRPMSub.close();
        passHoodAngleSub.close();
    }
}