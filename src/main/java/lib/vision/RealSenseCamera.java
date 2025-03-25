package lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;

/**
 * A wrapper class that provides access to the published Real Sense Camera data
 * as an object
 */
public class RealSenseCamera {

  private final String CamName;
  private NetworkTable mainTable;
  private NetworkTable debugTable;
  private Pose2d reefPose;
  private final DoubleArrayPublisher dblArrayPub;

  public RealSenseCamera(String name) {
    CamName = name;
    mainTable = NetworkTableInstance.getDefault().getTable(CamName);
    debugTable = NetworkTableInstance.getDefault().getTable("Camera Debug Table");
    dblArrayPub = debugTable.getDoubleArrayTopic("Stored Reef Pose").publish();

    reefPose = new Pose2d();
  }

  /**
   * Returns the current reef pose from network tables
   */
  public <Optional> Pose2d getReefPose() {
    return reefPose;
  }

  /**
   * Publishes the double array
   */
  public void publishArray(double[] values) {
    dblArrayPub.set(values);
  }

  /**
   * Updates reef pose from network tables
   */
  public void updateReefPose() {
    DoubleArrayEntry poseEntry = mainTable.getDoubleArrayTopic("ReefPose").getEntry(new double[0]);
    publishArray(poseEntry.get());

    TimestampedDoubleArray tsValue = poseEntry.getAtomic();
    double[] poseArray = tsValue.value;
    long timestamp = tsValue.timestamp;
    Translation2d tran2d = new Translation2d(poseArray[0], poseArray[1]);

    reefPose = new Pose2d(tran2d, new Rotation2d());
  }

}
