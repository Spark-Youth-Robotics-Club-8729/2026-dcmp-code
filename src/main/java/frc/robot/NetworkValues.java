package frc.robot;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.BooleanSubscriber;

public class NetworkValues {
    private static NetworkValues instance;
    NetworkTableInstance inst;
    private BooleanSubscriber fieldRelativeSub;
    private final Field2d m_field;

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

        DoubleTopic flywheelTopic = table.getDoubleTopic("flywheelRPM");
        flywheelRPMPub = flywheelTopic.publish();
        flywheelRPMPub.set(flywheelRPM);
        flywheelRPMSub = flywheelTopic.subscribe(flywheelRPM);

        DoubleTopic defaultFlywheelTopic = table.getDoubleTopic("defaultFlywheelRPM");
        defaultFlywheelRPMPub = defaultFlywheelTopic.publish();
        defaultFlywheelRPMPub.set(defaultFlywheelRPM);
        defaultFlywheelRPMSub = defaultFlywheelTopic.subscribe(defaultFlywheelRPM);

        BooleanTopic fieldRelativeTopic = table.getBooleanTopic("fieldRelative");
        fieldRelativeSub = fieldRelativeTopic.subscribe(true);


        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);
    }

    public double GetMaxSpeedInMeters()
    {
        return maxSpeedInMeters;
    }

    public double getFlywheelRPM() {
        return flywheelRPM;
    }

    public double getDefaultFlywheelRPM() {
        return defaultFlywheelRPM;
    }

    public boolean isFieldRelative() {
        return fieldRelativeSub.get();
    }

    public void periodic(Pose2d robotPose) {
        defaultFlywheelRPM = defaultFlywheelRPMSub.get();
        flywheelRPM = flywheelRPMSub.get();
        maxSpeedInMeters = maxSpeedInMetersSub.get();

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
    }
}