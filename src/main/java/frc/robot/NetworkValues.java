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
import edu.wpi.first.math.geometry.Pose2d;

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

        // field and offense/defense
        offenseModeSub = table.getBooleanTopic("offenseMode").subscribe(true);

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

    public void periodic(Pose2d robotPose) {
        defaultFlywheelRPM = defaultFlywheelRPMSub.get();
        flywheelRPM = flywheelRPMSub.get();
        maxSpeedInMeters = maxSpeedInMetersSub.get();
        passHoodAngle = passHoodAngleSub.get();
        offenseMode = offenseModeSub.get();

        m_field.setRobotPose(robotPose);
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