package lib.vision;

import java.util.function.Consumer;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanId.Swerve;
import lib.vision.LimelightHelpers.PoseEstimate;
import lib.vision.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * A wrapper class that provides access to a Limelight camera as an object
 */
public class Limelight {

  private final String LLName;
  private boolean doRejectUpdate;
  private final NetworkTable mainTable;
  private final NetworkTable subTable;
  private Matrix<N3, N1> stdDevs;
  private Matrix<N3, N1> maxStdDevs;
  private StructPublisher<Pose2d> pose2dpublisher;

  public Limelight(String name) {
    LLName = name;
    mainTable = NetworkTableInstance.getDefault().getTable("Limelight Data");
    subTable = mainTable.getSubTable(LLName);
    pose2dpublisher = subTable.getStructTopic("Pose2D", Pose2d.struct).publish();
    doRejectUpdate = false;
    stdDevs = Constants.Vision.kDefaultSingleTagStdDevs;
    maxStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
  }

  /** Sets robot orientation used by the MegaTag2 pose estimation algorithm */
  public void setRobotOrientation(double yaw, double yawRate, double pitch, double pitchRate, double roll,
      double rollRate) {

    LimelightHelpers.SetRobotOrientation(LLName,
        yaw, yawRate, pitch, pitchRate, roll, rollRate);
  }

  /**
   * DO NOT change to red since ALWAYS blue origin
   */
  public LimelightHelpers.PoseEstimate getBotPoseEstimate() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LLName);
  }

  public void setPoseNT(LimelightHelpers.PoseEstimate poseEstimate) {
    pose2dpublisher.set(poseEstimate.pose);
  }

  public Matrix<N3, N1> calculateStdDevs(LimelightHelpers.PoseEstimate poseEstimate) {

    int numTargets = LimelightHelpers.getTargetCount(LLName);
    double avgDist = poseEstimate.avgTagDist;

    // Decrease std devs if multiple targets are visible
    avgDist /= (double) numTargets;
    if (numTargets > 1) {
      stdDevs = Constants.Vision.kDefaultMultiTagStdDevs;
    }

    // Increase std devs based on (average) distance
    if (numTargets == 1 && avgDist > 4) {
      // Distance greater than 4 meters, and only one tag detected, resort to maximum
      // std devs
      stdDevs = maxStdDevs;
    } else {
      stdDevs = stdDevs.times(1 + (avgDist * avgDist / 30));
    }
    return stdDevs;
  }

}
