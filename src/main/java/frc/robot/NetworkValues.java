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
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class NetworkValues {
    private static NetworkValues instance;
    NetworkTableInstance inst;
    private final Field2d m_field;

    // PASSING (ALLIANCE)
    private DoublePublisher passHoodAnglePub;
    private DoubleSubscriber passHoodAngleSub;
    private double passHoodAngle = ShooterConstants.kDefaultPassHoodAngle;

    private DoublePublisher passRPMPub;
    private DoubleSubscriber passRPMSub;
    private double passRPM = ShooterConstants.maxFlywheelSpeedRPM;

    // PASSING (CROSS-COURT)
    private DoublePublisher courtHoodAnglePub;
    private DoubleSubscriber courtHoodAngleSub;
    private double courtHoodAngle = 26.0; 

    private DoublePublisher courtRPMPub;
    private DoubleSubscriber courtRPMSub;
    private double courtRPM = ShooterConstants.maxFlywheelSpeedRPM;

    // OFFENSE / DEFENSE TOGGLE
    private BooleanSubscriber offenseModeSub;
    private boolean offenseMode = true;

    // HUB SHOT
    private DoublePublisher hubHoodAnglePub;
    private DoubleSubscriber hubHoodAngleSub;
    private double hubHoodAngle = ShooterConstants.hubHoodAngleDeg;

    private DoublePublisher hubRPMPub;
    private DoubleSubscriber hubRPMSub;
    private double hubRPM = ShooterConstants.hubFlywheelRPM;

    // TOWER SHOT
    private DoublePublisher towerHoodAnglePub;
    private DoubleSubscriber towerHoodAngleSub;
    private double towerHoodAngle = ShooterConstants.towerHoodAngleDeg;

    private DoublePublisher towerRPMPub;
    private DoubleSubscriber towerRPMSub;
    private double towerRPM = ShooterConstants.towerFlywheelRPM;

    // DRIVE
    private DoublePublisher maxSpeedInMetersPub;
    private DoubleSubscriber maxSpeedInMetersSub;
    private double maxSpeedInMeters = DriveConstants.kMaxSpeedMetersPerSecond;

    // live tune normal flywheel RPM
    private DoublePublisher defaultFlywheelRPMPub;
    private DoubleSubscriber defaultFlywheelRPMSub;
    private double defaultFlywheelRPM = ShooterConstants.defaultFlywheelSpeedRPM;

    // live tune max flywheel RPM
    DoublePublisher flywheelRPMPub;
    DoubleSubscriber flywheelRPMSub;
    double flywheelRPM = ShooterConstants.maxFlywheelSpeedRPM;

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

        DoubleTopic defaultRPMTopic = table.getDoubleTopic("defaultFlywheelRPM");
        defaultFlywheelRPMPub = defaultRPMTopic.publish();
        defaultFlywheelRPMPub.set(defaultFlywheelRPM);
        defaultFlywheelRPMSub = defaultRPMTopic.subscribe(defaultFlywheelRPM);

        // alliance pass
        DoubleTopic passHoodTopic = table.getDoubleTopic("passHoodAngleDeg");
        passHoodAnglePub = passHoodTopic.publish();
        passHoodAnglePub.set(passHoodAngle);
        passHoodAngleSub = passHoodTopic.subscribe(passHoodAngle);

        DoubleTopic passRPMTopic = table.getDoubleTopic("passRPM");
        passRPMPub = passRPMTopic.publish();
        passRPMPub.set(passRPM);
        passRPMSub = passRPMTopic.subscribe(passRPM);

        // cross court pass
        DoubleTopic courtHoodTopic = table.getDoubleTopic("courtHoodAngleDeg");
        courtHoodAnglePub = courtHoodTopic.publish();
        courtHoodAnglePub.set(courtHoodAngle);
        courtHoodAngleSub = courtHoodTopic.subscribe(courtHoodAngle);
        
        DoubleTopic courtRPMTopic = table.getDoubleTopic("courtRPM"); 
        courtRPMPub = courtRPMTopic.publish(); 
        courtRPMPub.set(courtRPM);
        courtRPMSub = courtRPMTopic.subscribe(courtRPM);

        // hub shot
        DoubleTopic hubHoodTopic = table.getDoubleTopic("hubHoodAngleDeg");
        hubHoodAnglePub = hubHoodTopic.publish();
        hubHoodAnglePub.set(hubHoodAngle);
        hubHoodAngleSub = hubHoodTopic.subscribe(hubHoodAngle);

        DoubleTopic hubRPMTopic = table.getDoubleTopic("hubRPM");
        hubRPMPub = hubRPMTopic.publish();
        hubRPMPub.set(hubRPM);
        hubRPMSub = hubRPMTopic.subscribe(hubRPM);

        // tower shot
        DoubleTopic towerHoodTopic = table.getDoubleTopic("towerHoodAngleDeg");
        towerHoodAnglePub = towerHoodTopic.publish();
        towerHoodAnglePub.set(towerHoodAngle);
        towerHoodAngleSub = towerHoodTopic.subscribe(towerHoodAngle);

        DoubleTopic towerRPMTopic = table.getDoubleTopic("towerRPM");
        towerRPMPub = towerRPMTopic.publish();
        towerRPMPub.set(towerRPM);
        towerRPMSub = towerRPMTopic.subscribe(towerRPM);

        //flywheel
        DoubleTopic flywheelTopic = table.getDoubleTopic("passing RPM");
        flywheelRPMPub = flywheelTopic.publish();
        flywheelRPMPub.set(flywheelRPM);
        flywheelRPMSub = flywheelTopic.subscribe(flywheelRPM);

        DoubleTopic defaultFlywheelTopic = table.getDoubleTopic("defaultFlywheelRPM");
        defaultFlywheelRPMPub = defaultFlywheelTopic.publish();
        defaultFlywheelRPMPub.set(defaultFlywheelRPM);
        defaultFlywheelRPMSub = defaultFlywheelTopic.subscribe(defaultFlywheelRPM);

        //offense/defense
        offenseModeSub = table.getBooleanTopic("offenseMode").subscribe(true);

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

    public double getPassHoodAngle() {
        return passHoodAngle;
    }
    public double getPassRPM() {
        return passRPM;
    }

    public double getCourtHoodAngle() {
        return courtHoodAngle; 
    }
    public double getCourtRPM() {
        return courtRPM; 
    }

    public double getHubHoodAngle() {
        return hubHoodAngle; 
    }
    public double getHubRPM() {
        return hubRPM; 
    }

    public double getTowerHoodAngle() {
        return towerHoodAngle; 
    }
    public double getTowerRPM() {
        return towerRPM; 
    }

    public void systemTest(Command command) {
        SmartDashboard.putData("System Test", command);
    }

    public void periodic(RobotContainer robotContainer) {

        defaultFlywheelRPM = defaultFlywheelRPMSub.get();
        flywheelRPM = flywheelRPMSub.get();
        maxSpeedInMeters = maxSpeedInMetersSub.get();
        offenseMode = offenseModeSub.get();

        passHoodAngle = passHoodAngleSub.get();
        passRPM = passRPMSub.get();

        courtHoodAngle = courtHoodAngleSub.get();
        courtRPM = courtRPMSub.get();

        hubHoodAngle = hubHoodAngleSub.get();
        hubRPM = hubRPMSub.get();

        towerHoodAngle = towerHoodAngleSub.get();
        towerRPM = towerRPMSub.get();

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