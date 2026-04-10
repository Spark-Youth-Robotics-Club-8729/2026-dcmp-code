package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.FieldConstants;
import java.util.Set;
import java.util.function.Supplier;

public class VisionSubsystem extends SubsystemBase {

  /** Functional interface to match your Drive::addVisionMeasurement signature */
  @FunctionalInterface
  public interface VisionConsumer {
    void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
  }

  // Vision state variables
  private boolean connected = false;
  private int tagCount = 0;
  private double tx = 0.0;
  private double ty = 0.0;
  private double avgTagDist = 0.0;
  private int[] visibleIds = new int[0];
  private Pose2d mt2Pose = new Pose2d();
  private double timestamp = 0.0;
  private boolean enabledVisionUpdatesPose = false;

  // variable. its created once code is built, so it will work
  private final Supplier<Rotation2d> gyroSupplier;
  private final VisionConsumer odometryConsumer; // Ri3D used something similar to give both the Pose and the timestamp


  //default constructor
  public VisionSubsystem() {
    this.gyroSupplier = () -> Rotation2d.fromDegrees(0); // default to 0 degrees if no supplier is provided
    this.odometryConsumer = (pose, timestamp, stdDevs) -> {}; // default to no-op if no consumer is provided
    initializeLimelight();
  }

  //constructor to use later if wanting to implement pose estimation with vision
  //it will take in a supplier with the current gyro angle, pose, and timestamp
  public VisionSubsystem(Supplier<Rotation2d> gyroSupplier, VisionConsumer odometryConsumer) {
    this.gyroSupplier = gyroSupplier;
    this.odometryConsumer = odometryConsumer;
    initializeLimelight();
  }

  private void initializeLimelight() {
    // LL4 will now only use the unblocked ids (we pass the validIDs and ignore the
    // blocked ones)
    int[] validIds = Constants.VisionConstants.LAYOUT.getTags().stream()
        .mapToInt(tag -> tag.ID)
        .filter(id -> !Constants.VisionConstants.BLOCKED_TAG_IDS.contains(id))
        .toArray();
    LimelightHelpers.SetFiducialIDFiltersOverride(Constants.VisionConstants.CAMERA_NAME, validIds);

    // Seed the LL IMU on startup (means to set the LL4 IMU to the robot gyro)
    LimelightHelpers.SetIMUMode(
        Constants.VisionConstants.CAMERA_NAME, 1); // we do this because LL4 has an IMU, while older versions dont
  }

  @Override
  public void periodic() {
    updateInputs();

    // update Odometry using MegaTag 2 ONLY if enabled and data is valid
    if (enabledVisionUpdatesPose && tagCount >= Constants.VisionConstants.MT2_MIN_TAGS && connected) {
      // Use standard deviation because it will prevent the vision from jumping
      double xStdDev = 0.1 * Math.pow(avgTagDist, 2) / tagCount;
      double yStdDev = 0.1 * Math.pow(avgTagDist, 2) / tagCount;
      double thetaStdDev = 0.5; // trust the robot gyro mainly

      odometryConsumer.accept(
          mt2Pose, timestamp, VecBuilder.fill(xStdDev, yStdDev, thetaStdDev));
    }

    // the below stuff is for the IMU of the LL4 itself (older limelights dont have
    // IMUs but LL4
    // does, which is why this is needed)
    if (DriverStation.isDisabled()) {
      LimelightHelpers.SetIMUMode(
          Constants.VisionConstants.CAMERA_NAME, 1); // Seed (means to set the LL4 IMU to the robot gyro)
    } else { // when enabled
      LimelightHelpers.SetIMUMode(
          Constants.VisionConstants.CAMERA_NAME, 4); // Internal + Assist (means uses both robot gyro and LL4 IMU)
    }
  }

  private void updateInputs() {
    // Get Gyro from robot and feed it to LL4
    double yaw = gyroSupplier.get().getDegrees(); // get robot's gyro yaw
    LimelightHelpers.SetRobotOrientation(
        Constants.VisionConstants.CAMERA_NAME,
        yaw,
        0,
        0,
        0,
        0,
        0); // sets the robot orientation with the yaw. the pitch, roll, and rate are 0
            // because they
    // dont change (unless robot is tipped over)
    var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
        Constants.VisionConstants.CAMERA_NAME); // since yaw is given, it is easy for LL4 to calcualte the x,y pose of
    // robot

    // Get all of the data from limelight and check if its valid, then update variables
    connected = LimelightHelpers.getTV(Constants.VisionConstants.CAMERA_NAME); // true if valid target is found
    tx = LimelightHelpers.getTX(Constants.VisionConstants.CAMERA_NAME);
    ty = LimelightHelpers.getTY(Constants.VisionConstants.CAMERA_NAME);
    
    if (mt2 != null) {
      tagCount = mt2.tagCount;
      avgTagDist = mt2.avgTagDist;
      mt2Pose = mt2.pose;
      timestamp = mt2.timestampSeconds;

      visibleIds = new int[mt2.rawFiducials.length];
      for (int i = 0; i < mt2.rawFiducials.length; i++) {
        visibleIds[i] = mt2.rawFiducials[i].id;
      }
    }
  }

  /** Toggle whether vision data interferes with the Pose Estimator */
  public void enabledVisionUpdatesPose(boolean enabled) {
    enabledVisionUpdatesPose = enabled;
  }

  /** Simple check if any tag is visible */
  public boolean hasTarget() {
    return tagCount > 0;
  }

  /** Specifically checks if we see a hub tag for OUR alliance */
  public boolean hasHubTarget() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue); // uses OUR alliance
    Set<Integer> hubTags = (alliance == Alliance.Red) ? Constants.VisionConstants.RED_HUB_TAGS : Constants.VisionConstants.BLUE_HUB_TAGS;

    for (int id : visibleIds) {
      if (hubTags.contains(id))
        return true;
    }
    return false;
  }

  /** Returns distance to nearest Hub Tag for shot calculation */
  public double getDistanceToHub() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    Set<Integer> hubTags = (alliance == Alliance.Red) ? Constants.VisionConstants.RED_HUB_TAGS
        : Constants.VisionConstants.BLUE_HUB_TAGS;

    var raw = LimelightHelpers.getRawFiducials(Constants.VisionConstants.CAMERA_NAME);
    double minFound = Double.NaN;

    if (raw == null || raw.length == 0)
      return minFound; // check if raw is null or empty first to avoid crashes

    for (var f : raw) {
      if (hubTags.contains(f.id)) {
        if (Double.isNaN(minFound) || f.distToRobot < minFound) {
          minFound = f.distToRobot;
        }
      }
    }
    return minFound;
  }

  public Translation2d getAllianceHubPosition() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (alliance == Alliance.Red) {
      return new Translation2d(FieldConstants.LinesVertical.oppHubCenter, FieldConstants.fieldWidth / 2.0);
    } else {
      return new Translation2d(FieldConstants.LinesVertical.hubCenter, FieldConstants.fieldWidth / 2.0);
    }
  }
}