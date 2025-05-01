package frc.robot.SwerveClasses;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
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
  private SwerveModulePosition[] currentSwerveModulePositions = new SwerveModulePosition[4];

  private DataLog log;

  public SwerveOdometry(SwerveSubsystem subsystem, SwerveDriveKinematics kinematics, Limelight leftLL,
      Limelight rightLL) {
    this.subsystem = subsystem;
    this.leftLL = leftLL;
    this.rightLL = rightLL;
    leftLLPoseEstimate = leftLL.getBotPoseEstimate();
    rightLLPoseEstimate = rightLL.getBotPoseEstimate();

    log = DataLogManager.getLog();

    doRejectLeftLLUpdate = false;
    doRejectRightLLUpdate = false;

    swerveKinematics = kinematics;

    for (int i = 0; i < 4; i++) {
        currentSwerveModulePositions[i] = new SwerveModulePosition(
            subsystem.getSwerveModule(i).getPosition(),
            new Rotation2d(subsystem.getSwerveModule(i).getEncoderPosition()));
    }
    poseEstimator = new SwerveDrivePoseEstimator(
        swerveKinematics,
        subsystem.getRobotRotation2dForOdometry(),
        currentSwerveModulePositions,
        new Pose2d(0, 0, subsystem.getRobotRotation2dForOdometry()));
  }

  private void updateSwerveModulePositions() {
    for (int i = 0; i < 4; i++) {
        SwerveModule module = subsystem.getSwerveModule(i);
        this.currentSwerveModulePositions[i].distanceMeters = module.getPosition();
        this.currentSwerveModulePositions[i].angle = new Rotation2d(module.getEncoderPosition());
    }
  }

  public void update() {
    updateSwerveModulePositions();
    poseEstimator.update(
        subsystem.getRobotRotation2dForOdometry(),
        currentSwerveModulePositions);

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

    double poseRotation = getPoseEstimatedRotation();

    leftLL.setRobotOrientation(poseRotation, 0, 0, 0, 0, 0);
    rightLL.setRobotOrientation(poseRotation, 0, 0, 0, 0, 0);
    leftLLPoseEstimate = leftLL.getBotPoseEstimate();
    rightLLPoseEstimate = rightLL.getBotPoseEstimate();

    doRejectLeftLLUpdate = false;
    doRejectRightLLUpdate = false;

    // if our angular velocity is greater than 360 degrees per second, ignore vision
    // updates. 360 is the default value in docs
    if (Math.abs(subsystem.getAngularChassisSpeed()) > Constants.Vision.kMaxRotationRate.getValue()) {
      doRejectLeftLLUpdate = true;
      doRejectRightLLUpdate = true;
    }

    /**
     * Logic for updating poseEstimator based on left limelight
     */
    if (!LimelightHelpers.validPoseEstimate(leftLLPoseEstimate)) {
      doRejectLeftLLUpdate = true;
    }
    if (!doRejectLeftLLUpdate) {
      poseEstimator.setVisionMeasurementStdDevs(leftLL.calculateStdDevs(leftLLPoseEstimate));
      poseEstimator.addVisionMeasurement(
          leftLLPoseEstimate.pose,
          leftLLPoseEstimate.timestampSeconds);
      leftLL.setPoseNT(leftLLPoseEstimate);
    }

    /**
     * Logic for updating poseEstimator based on right limelight
     */

    if (!LimelightHelpers.validPoseEstimate(rightLLPoseEstimate)) {
      doRejectRightLLUpdate = true;
    }
    if (!doRejectRightLLUpdate) {
      poseEstimator.setVisionMeasurementStdDevs(rightLL.calculateStdDevs(rightLLPoseEstimate));
      poseEstimator.addVisionMeasurement(
          rightLLPoseEstimate.pose,
          rightLLPoseEstimate.timestampSeconds);
      rightLL.setPoseNT(rightLLPoseEstimate);
    }
  }

  public double getPoseEstimatedRotation() {
    return poseEstimator.getEstimatedPosition().getRotation().getDegrees();
  }

  public Pose2d getEstimatedPosition() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPosition() {
    updateSwerveModulePositions();
    poseEstimator.resetPosition(
        subsystem.getRobotRotation2dForOdometry(),
        currentSwerveModulePositions,
        new Pose2d(0, 0, subsystem.getRobotRotation2dForOdometry()));
  }

  public void setPosition(Pose2d pos) {

    SmartDashboard.putNumber("Pathplanner Angle", pos.getRotation().getRadians());
    SmartDashboard.putNumber("Setting Angle", subsystem.getRobotRotation2dForOdometry().getRadians());

    updateSwerveModulePositions();
    poseEstimator.resetPosition(
        subsystem.getRobotRotation2dForOdometry(),
        currentSwerveModulePositions,
        pos);
  }
}
