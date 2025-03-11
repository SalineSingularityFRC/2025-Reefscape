package frc.robot.SwerveClasses;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import lib.vision.Limelight;
import lib.vision.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveOdometry {
  SwerveDrivePoseEstimator poseEstimator;

  private final int FL = 0;
  private final int FR = 1;
  private final int BL = 2;
  private final int BR = 3;

  private Limelight leftLL;
  private Limelight rightLL;
  private boolean doRejectLeftLLUpdate;
  private boolean doRejectRightLLUpdate;
  private LimelightHelpers.PoseEstimate leftLLPoseEstimate;
  private LimelightHelpers.PoseEstimate rightLLPoseEstimate;

  private final SwerveDriveKinematics swerveKinematics;

  private SwerveSubsystem subsystem;

  private DataLog log;

  public SwerveOdometry(SwerveSubsystem subsystem, Translation2d[] vectorKinematics, Limelight leftLL,
      Limelight rightLL) {
    this.subsystem = subsystem;
    this.leftLL = leftLL;
    this.rightLL = rightLL;
    leftLLPoseEstimate = leftLL.getBotPoseEstimate();
    rightLLPoseEstimate = rightLL.getBotPoseEstimate();

    log = DataLogManager.getLog();

    doRejectLeftLLUpdate = false;
    doRejectRightLLUpdate = false;

    swerveKinematics = new SwerveDriveKinematics(
        vectorKinematics[FL], vectorKinematics[FR], vectorKinematics[BL], vectorKinematics[BR]);

    poseEstimator = new SwerveDrivePoseEstimator(
        swerveKinematics,
        subsystem.getRobotRotation2dForOdometry(),
        new SwerveModulePosition[] {
            new SwerveModulePosition(
                subsystem.getSwerveModule(FL).getPosition(),
                new Rotation2d(subsystem.getSwerveModule(FL).getEncoderPosition())),
            new SwerveModulePosition(
                subsystem.getSwerveModule(FR).getPosition(),
                new Rotation2d(subsystem.getSwerveModule(FR).getEncoderPosition())),
            new SwerveModulePosition(
                subsystem.getSwerveModule(BL).getPosition(),
                new Rotation2d(subsystem.getSwerveModule(BL).getEncoderPosition())),
            new SwerveModulePosition(
                subsystem.getSwerveModule(BR).getPosition(),
                new Rotation2d(subsystem.getSwerveModule(BR).getEncoderPosition())),
        },
        new Pose2d(0, 0, subsystem.getRobotRotation2dForOdometry()));
  }

  public void update() {
    poseEstimator.update(
        subsystem.getRobotRotation2dForOdometry(),
        new SwerveModulePosition[] {
            new SwerveModulePosition(
                subsystem.getSwerveModule(FL).getPosition(),
                new Rotation2d(subsystem.getSwerveModule(FL).getEncoderPosition())),
            new SwerveModulePosition(
                subsystem.getSwerveModule(FR).getPosition(),
                new Rotation2d(subsystem.getSwerveModule(FR).getEncoderPosition())),
            new SwerveModulePosition(
                subsystem.getSwerveModule(BL).getPosition(),
                new Rotation2d(subsystem.getSwerveModule(BL).getEncoderPosition())),
            new SwerveModulePosition(
                subsystem.getSwerveModule(BR).getPosition(),
                new Rotation2d(subsystem.getSwerveModule(BR).getEncoderPosition())),
        });

    addLLVisionMeasurement();

    // Assuming DataLogManager has already been started and log initialized
    // DoubleLogEntry targetXLog = new DoubleLogEntry(log, "Target X");
    // DoubleLogEntry targetYLog = new DoubleLogEntry(log, "Target Y");
    // DoubleLogEntry targetZLog = new DoubleLogEntry(log, "Target Z");
    // DoubleLogEntry targetPitchLog = new DoubleLogEntry(log, "Target Pitch");
    // DoubleLogEntry targetYawLog = new DoubleLogEntry(log, "Target Yaw");
    // DoubleLogEntry targetRollLog = new DoubleLogEntry(log, "Target Roll");

    // double[] botPose = LimelightHelpers.getBotPose_TargetSpace("limelight");
    // targetXLog.append(botPose[0]);
    // targetYLog.append(botPose[1]);
    // targetZLog.append(botPose[2]);
    // targetPitchLog.append(botPose[3]);
    // targetYawLog.append(botPose[4]);
    // targetRollLog.append(botPose[5]);

    // SmartDashboard.putNumber("Target X",
    // LimelightHelpers.getBotPose_TargetSpace("limelight")[0]);
    // SmartDashboard.putNumber("Target Y",
    // LimelightHelpers.getBotPose_TargetSpace("limelight")[1]);
    // SmartDashboard.putNumber("Target Z",
    // LimelightHelpers.getBotPose_TargetSpace("limelight")[2]);
    // SmartDashboard.putNumber("Target Pitch",
    // LimelightHelpers.getBotPose_TargetSpace("limelight")[3]);
    // SmartDashboard.putNumber("Target Yaw",
    // LimelightHelpers.getBotPose_TargetSpace("limelight")[4]);
    // SmartDashboard.putNumber("Target Roll",
    // LimelightHelpers.getBotPose_TargetSpace("limelight")[5]);
  }

  /**
   * Adds limelight data to the kalman filter
   */
  public void addLLVisionMeasurement() {
    leftLL.setRobotOrientation(poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    rightLL.setRobotOrientation(poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    leftLLPoseEstimate = leftLL.getBotPoseEstimate();
    rightLLPoseEstimate = rightLL.getBotPoseEstimate();

    /**
     * Logic for updating poseEstimator based on left limelight
     */
    doRejectLeftLLUpdate = false;

    // if our angular velocity is greater than 680 degrees per second, ignore vision
    // updates
    if (Math.abs(subsystem.getAngularChassisSpeed()) > 680) {
      doRejectLeftLLUpdate = true;
    }
    if (leftLLPoseEstimate.tagCount == 0) {
      doRejectLeftLLUpdate = true;
    }
    if (!doRejectLeftLLUpdate) {
      poseEstimator.setVisionMeasurementStdDevs(leftLL.calculateStdDevs(leftLLPoseEstimate));
      poseEstimator.addVisionMeasurement(
          leftLLPoseEstimate.pose,
          leftLLPoseEstimate.timestampSeconds);
    }

    leftLL.setPoseNT(leftLLPoseEstimate);

    /**
     * Logic for updating poseEstimator based on right limelight
     */
    doRejectRightLLUpdate = false;

    // if our angular velocity is greater than 680 degrees per second, ignore vision
    // updates
    if (Math.abs(subsystem.getAngularChassisSpeed()) > 680) {
      doRejectRightLLUpdate = true;
    }
    if (rightLLPoseEstimate.tagCount == 0) {
      doRejectRightLLUpdate = true;
    }
    if (!doRejectRightLLUpdate) {
      poseEstimator.setVisionMeasurementStdDevs(rightLL.calculateStdDevs(leftLLPoseEstimate));
      poseEstimator.addVisionMeasurement(
        rightLLPoseEstimate.pose,
        rightLLPoseEstimate.timestampSeconds);
    }

    rightLL.setPoseNT(leftLLPoseEstimate);
  }

  public Pose2d getEstimatedPosition() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPosition() {
    poseEstimator.resetPosition(
        subsystem.getRobotRotation2dForOdometry(),
        new SwerveModulePosition[] {
            new SwerveModulePosition(
                subsystem.getSwerveModule(FL).getPosition(),
                new Rotation2d(subsystem.getSwerveModule(FL).getEncoderPosition())),
            new SwerveModulePosition(
                subsystem.getSwerveModule(FR).getPosition(),
                new Rotation2d(subsystem.getSwerveModule(FR).getEncoderPosition())),
            new SwerveModulePosition(
                subsystem.getSwerveModule(BL).getPosition(),
                new Rotation2d(subsystem.getSwerveModule(BL).getEncoderPosition())),
            new SwerveModulePosition(
                subsystem.getSwerveModule(BR).getPosition(),
                new Rotation2d(subsystem.getSwerveModule(BR).getEncoderPosition())),
        },
        new Pose2d(0, 0, subsystem.getRobotRotation2dForOdometry()));
  }

  public void setPosition(Pose2d pos) {

    SmartDashboard.putNumber("Pathplanner Angle", pos.getRotation().getRadians());
    SmartDashboard.putNumber("Setting Angle", subsystem.getRobotRotation2dForOdometry().getRadians());

    poseEstimator.resetPosition(
        subsystem.getRobotRotation2dForOdometry(),
        new SwerveModulePosition[] {
            new SwerveModulePosition(
                subsystem.getSwerveModule(FL).getPosition(),
                new Rotation2d(subsystem.getSwerveModule(FL).getEncoderPosition())),
            new SwerveModulePosition(
                subsystem.getSwerveModule(FR).getPosition(),
                new Rotation2d(subsystem.getSwerveModule(FR).getEncoderPosition())),
            new SwerveModulePosition(
                subsystem.getSwerveModule(BL).getPosition(),
                new Rotation2d(subsystem.getSwerveModule(BL).getEncoderPosition())),
            new SwerveModulePosition(
                subsystem.getSwerveModule(BR).getPosition(),
                new Rotation2d(subsystem.getSwerveModule(BR).getEncoderPosition())),
        },
        pos);
  }
}
