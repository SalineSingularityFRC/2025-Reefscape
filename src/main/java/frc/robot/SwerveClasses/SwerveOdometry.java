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
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveOdometry {
  SwerveDrivePoseEstimator swerveOdometry;

  private final int FL = 0;
  private final int FR = 1;
  private final int BL = 2;
  private final int BR = 3;

  private final SwerveDriveKinematics swerveKinematics;

  private SwerveSubsystem subsystem;

  private DataLog log;

  private StructPublisher<Pose2d> publisher_left_limelight;
  private StructPublisher<Pose2d> publisher_right_limelight;

  private NetworkTableInstance inst;
  private NetworkTable table;

  Matrix<N3, N1> stdDevs;
  Matrix<N3, N1> maxStdDevs;

  private Boolean BlueAlliance;

  public SwerveOdometry(SwerveSubsystem subsystem, Translation2d[] vectorKinematics) {
    this.subsystem = subsystem;

    log = DataLogManager.getLog();

    var Alliance = DriverStation.getAlliance();
    BlueAlliance = true;
    if (Alliance.isPresent()) {
      if (Alliance.get() == DriverStation.Alliance.Red) {
        BlueAlliance = false;
      }
    }

    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("datatable");
    publisher_left_limelight = table.getStructTopic("Limelight MegaTag2 Pos Left", Pose2d.struct).publish();
    publisher_right_limelight = table.getStructTopic("Limelight MegaTag2 Pos Right", Pose2d.struct).publish();

    swerveKinematics = new SwerveDriveKinematics(
        vectorKinematics[FL], vectorKinematics[FR], vectorKinematics[BL], vectorKinematics[BR]);

    swerveOdometry = new SwerveDrivePoseEstimator(
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

    stdDevs = Constants.Vision.kDefaultSingleTagStdDevs;
    maxStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
  }

  public void update() {
    swerveOdometry.update(
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

    boolean doRejectUpdate = false;

    // MegaTag 2
    LimelightHelpers.SetRobotOrientation("limelight-right",
        swerveOdometry.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-top",
        swerveOdometry.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);

    // Always wpiBlue no matter what since we are always using blue origin
    LimelightHelpers.PoseEstimate mt2_right = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
    LimelightHelpers.PoseEstimate mt2_left = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-top");

    if (Math.abs(subsystem.getAngularChassisSpeed()) > 680) // if our angular velocity is greater than 680 degrees per
                                                            // second, ignore vision updates
    {
      doRejectUpdate = true;
    }
    if (mt2_left.tagCount == 0) {
      doRejectUpdate = true;
    }
    if (!doRejectUpdate) {
      swerveOdometry.setVisionMeasurementStdDevs(calculateStdDevs(mt2_left));
      swerveOdometry.addVisionMeasurement(
          mt2_left.pose,
          mt2_left.timestampSeconds);
    }

    if (Math.abs(subsystem.getAngularChassisSpeed()) > 680) // if our angular velocity is greater than 680 degrees per
                                                            // second, ignore vision updates
    {
      doRejectUpdate = true;
    }
    if (mt2_right.tagCount == 0) {
      doRejectUpdate = true;
    }
    if (!doRejectUpdate) {
      swerveOdometry.setVisionMeasurementStdDevs(calculateStdDevs(mt2_right));
      swerveOdometry.addVisionMeasurement(
          mt2_right.pose,
          mt2_right.timestampSeconds);
    }

    publisher_left_limelight.set(mt2_left.pose);
    publisher_right_limelight.set(mt2_right.pose);

    // Assuming DataLogManager has already been started and log initialized
    DoubleLogEntry targetXLog = new DoubleLogEntry(log, "Target X");
    DoubleLogEntry targetYLog = new DoubleLogEntry(log, "Target Y");
    DoubleLogEntry targetZLog = new DoubleLogEntry(log, "Target Z");
    DoubleLogEntry targetPitchLog = new DoubleLogEntry(log, "Target Pitch");
    DoubleLogEntry targetYawLog = new DoubleLogEntry(log, "Target Yaw");
    DoubleLogEntry targetRollLog = new DoubleLogEntry(log, "Target Roll");

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

  public Pose2d getEstimatedPosition() {
    return swerveOdometry.getEstimatedPosition();
  }

  private Matrix<N3, N1> calculateStdDevs(LimelightHelpers.PoseEstimate poseEstimate) {

    int numTargets = poseEstimate.tagCount;
    double avgDist = poseEstimate.avgTagDist;

    // Decrease std devs if multiple targets are visible
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

  public void resetPosition() {
    swerveOdometry.resetPosition(
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

    swerveOdometry.resetPosition(
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
