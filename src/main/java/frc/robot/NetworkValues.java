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
import edu.wpi.first.math.geometry.Pose2d;

public class NetworkValues {
    private static NetworkValues instance;
    NetworkTableInstance inst;

    private final Field2d m_field;

    DoublePublisher maxSpeedInMetersPub;
    DoubleSubscriber maxSpeedInMetersSub;
    double maxSpeedInMeters = DriveConstants.kMaxSpeedMetersPerSecond;

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
        maxSpeedInMetersPub.set(0.5);
        maxSpeedInMetersSub = maxSpeedInMetersTopic.subscribe(maxSpeedInMeters);

        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);
    }

    public double GetMaxSpeedInMeters()
    {
        return maxSpeedInMeters;
    }

    public void periodic(Pose2d robotPose) {
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
    }
}