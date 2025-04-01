package lib.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * A wrapper class that provides access to the published Real Sense Camera data
 * as an object
 */
public class RealSenseCamera {

  private final String CamName;
  private NetworkTable mainTable;
  private NetworkTable debugTable;
  private NetworkTable poseInfoTable;
  private DoubleEntry xPoseEntry;
  private DoubleEntry yPoseEntry;
  private Pose2d reefPose, lastPose;
  private final DoubleArrayPublisher dblArrayPub;
  private final int threshold = 8;
  private int stableCount = 0;

  public RealSenseCamera(String name) {
    CamName = name;
    mainTable = NetworkTableInstance.getDefault().getTable(CamName);
    poseInfoTable = mainTable.getSubTable("L4");
    xPoseEntry = poseInfoTable.getDoubleTopic("0").getEntry(-1);
    yPoseEntry = poseInfoTable.getDoubleTopic("1").getEntry(-1);

    // Not needed
    debugTable = NetworkTableInstance.getDefault().getTable("Camera Debug Table");
    dblArrayPub = debugTable.getDoubleArrayTopic("Stored Reef Pose").publish();

    reefPose = new Pose2d();
    lastPose = new Pose2d();
  }

  /**
   * Returns the current reef pose from network tables
   */
  public Supplier<Pose2d> getReefPose() {
    return () -> reefPose;
  }

  /**
   * Publishes the double array
   */
  public void publishArray(double[] values) {
    dblArrayPub.set(values);
    Flush();
  }

  public static void Flush() {
    NetworkTableInstance.getDefault().flush();
  }

  /**
   * Updates reef pose from network tables
   */
  public void updateReefPose() {
    // DoubleArrayEntry poseEntry = mainTable.getDoubleArrayTopic("L4").getEntry(new double[0]);

    // // Can effectively ignore this for now
    // publishArray(poseEntry.get());

    // TimestampedDoubleArray tsValue = poseEntry.getAtomic();
    // double[] poseArray = tsValue.value;
    // long timestamp = tsValue.timestamp;
    // Translation2d tran2d = new Translation2d(poseArray[0], poseArray[1]);

    Translation2d trans2d = new Translation2d(xPoseEntry.get(), yPoseEntry.get());
    reefPose = new Pose2d(trans2d, new Rotation2d());
  }

  /**
   * Checks if the camera pose has consistently been under a set tolerance
   * @return If the camera pose was under a set tolerance for a set threshold
   */
  public boolean isCameraPoseStable() {
    if (reefPose.getTranslation().getDistance(lastPose.getTranslation()) < Constants.Drive.L4_PID_DRIVE_POSE_TOLERANCE.getValue()
        && !reefPose.getTranslation().equals(lastPose.getTranslation())) {
      stableCount++;
    } else {
      stableCount = 0;
    }
    lastPose = reefPose;
    return stableCount >= Constants.Drive.L4_PID_DRIVE_STABLE_COUNT_THRESHOLD.getValue();
  }
}
