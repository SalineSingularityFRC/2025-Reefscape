package lib.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private DoubleArrayTopic poseInfoDoubleArrayTopic;
  private DoubleArrayEntry poseEntry;
  private DoubleTopic timestampTopic;
  private DoubleEntry timestampEntry;
  private Pose2d finalReefPose;
  private Translation2d lastTranslation2d;
  private final DoubleArrayPublisher dblArrayPub;
  private int stableCount = 0;
  private double lastTimestamp;
  private Timer timer;

  public RealSenseCamera(String name) {
    CamName = name;
    mainTable = NetworkTableInstance.getDefault().getTable(CamName);
    poseInfoDoubleArrayTopic = mainTable.getDoubleArrayTopic("L4");
    poseEntry = poseInfoDoubleArrayTopic.getEntry(new double[] { -1, -1 });

    timestampTopic = mainTable.getDoubleTopic("L4_t");
    timestampEntry = timestampTopic.getEntry(0);

    // Not needed
    debugTable = NetworkTableInstance.getDefault().getTable("Camera Debug Table");
    dblArrayPub = debugTable.getDoubleArrayTopic("Stored Reef Pose").publish();

    finalReefPose = new Pose2d();
    lastTranslation2d = new Translation2d();

    timer = new Timer();
    timer.start();
    lastTimestamp = 0;
  }

  /**
   * Returns the current reef pose from network tables
   */
  public Supplier<Pose2d> getFinalReefPose() {
    return () -> finalReefPose;
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
    // DoubleArrayEntry poseEntry = mainTable.getDoubleArrayTopic("L4").getEntry(new
    // double[0]);

    // // Can effectively ignore this for now
    // publishArray(poseEntry.get());

    // TimestampedDoubleArray tsValue = poseEntry.getAtomic();
    // double[] poseArray = tsValue.value;
    // long timestamp = tsValue.timestamp;
    // Translation2d tran2d = new Translation2d(poseArray[0], poseArray[1]);

    double[] poseEntryGet = poseEntry.get();
    Translation2d trans2d = new Translation2d(poseEntryGet[0], poseEntryGet[1]);
    SmartDashboard.putNumberArray("realsensecamera/poseEntryGet", poseEntryGet);

    double currentTimestamp = timestampEntry.get();
    SmartDashboard.putNumber("realsensecamera/currentTimestamp", currentTimestamp);
    SmartDashboard.putNumber("realsensecamera/timehaselapsed", timer.get());

    if (currentTimestamp != lastTimestamp) {
      timer.restart();
    }
    lastTimestamp = currentTimestamp;

    if (currentTimestamp == 0 || timer.hasElapsed(5) || !isCameraPoseStable(trans2d)) { // make constants thing later
      finalReefPose = null;
      SmartDashboard.putBoolean("realsensecamera/good", false); // MAKE CONSTANTS LATER
    } else {
      finalReefPose = new Pose2d(trans2d, new Rotation2d());
      SmartDashboard.putBoolean("realsensecamera/good", finalReefPose.getX() < 1); // MAKE CONSTANTS LATER
    }
  }

  /**
   * Checks if the camera pose has consistently been under a set tolerance
   * 
   * @return If the camera pose was under a set tolerance for a set threshold
   */
  private boolean isCameraPoseStable(Translation2d poseTranslation2d) {
    SmartDashboard.putNumber("realsensecamera/stableCount", stableCount);

    if (poseTranslation2d.getDistance(lastTranslation2d) < Constants.Drive.L4_PID_DRIVE_POSE_TOLERANCE
        .getValue()
        && !poseTranslation2d.equals(lastTranslation2d)) {
      stableCount++;
    } else {
      stableCount = 0;
    }
    lastTranslation2d = poseTranslation2d;
    return stableCount >= Constants.Drive.L4_PID_DRIVE_STABLE_COUNT_THRESHOLD.getValue();
  }
}
