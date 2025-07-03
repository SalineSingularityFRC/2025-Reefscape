package frc.robot.SwerveClasses;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.*;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import lib.vision.Limelight;
import lib.vision.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;

public class SwerveOdometry {
  SwerveDrivePoseEstimator poseEstimator;

  private Limelight leftLL;
  private Limelight rightLL;
  private boolean doRejectLeftLLUpdate;
  private boolean doRejectRightLLUpdate;
  private LimelightHelpers.PoseEstimate leftLLPoseEstimate;
  private LimelightHelpers.PoseEstimate rightLLPoseEstimate;
  private static double gyroZero = SwerveSubsystem.gyroZero;
  private SwerveModule[] m_modules;
  private final SwerveDriveKinematics swerveKinematics;
  private static SwerveSubsystem swerveSubsystem;
  private SwerveModulePosition[] m_modulePositions = new SwerveModulePosition[4];

  private DataLog log;

  public SwerveOdometry(SwerveSubsystem swerveSubsystem, SwerveDriveKinematics kinematics, Limelight leftLL,
      Limelight rightLL) {
    this.swerveSubsystem = swerveSubsystem;
    this.leftLL = leftLL;
    this.rightLL = rightLL;
    leftLLPoseEstimate = leftLL.getBotPoseEstimate();
    rightLLPoseEstimate = rightLL.getBotPoseEstimate();

    log = DataLogManager.getLog();

    doRejectLeftLLUpdate = false;
    doRejectRightLLUpdate = false;

    swerveKinematics = kinematics;

    for (int i = 0; i < 4; i++) {
      m_modulePositions[i] = new SwerveModulePosition(
          swerveSubsystem.getSwerveModule(i).getPosition(),
          new Rotation2d(swerveSubsystem.getSwerveModule(i).getEncoderPosition()));
    }
    poseEstimator = new SwerveDrivePoseEstimator(
        swerveKinematics,
        OdometryThread.getFieldCorrectedAngle(),
        m_modulePositions,
        new Pose2d(0, 0, OdometryThread.getFieldCorrectedAngle()));
  }

  private void updateSwerveModulePositions() {
    for (int i = 0; i < 4; i++) {
      SwerveModule module = swerveSubsystem.getSwerveModule(i);
      this.m_modulePositions[i].distanceMeters = module.getPosition();
      this.m_modulePositions[i].angle = new Rotation2d(module.getEncoderPosition());
    }
  }

  // public void update() {

  // // Assuming DataLogManager has already been started and log initialized
  // // DoubleLogEntry targetXLog = new DoubleLogEntry(log, "Target X");
  // // DoubleLogEntry targetYLog = new DoubleLogEntry(log, "Target Y");
  // // DoubleLogEntry targetZLog = new DoubleLogEntry(log, "Target Z");
  // // DoubleLogEntry targetPitchLog = new DoubleLogEntry(log, "Target Pitch");
  // // DoubleLogEntry targetYawLog = new DoubleLogEntry(log, "Target Yaw");
  // // DoubleLogEntry targetRollLog = new DoubleLogEntry(log, "Target Roll");

  // // double[] botPose = LimelightHelpers.getBotPose_TargetSpace("limelight");
  // // targetXLog.append(botPose[0]);
  // // targetYLog.append(botPose[1]);
  // // targetZLog.append(botPose[2]);
  // // targetPitchLog.append(botPose[3]);
  // // targetYawLog.append(botPose[4]);
  // // targetRollLog.append(botPose[5]);

  // // SmartDashboard.putNumber("Target X",
  // // LimelightHelpers.getBotPose_TargetSpace("limelight")[0]);
  // // SmartDashboard.putNumber("Target Y",
  // // LimelightHelpers.getBotPose_TargetSpace("limelight")[1]);
  // // SmartDashboard.putNumber("Target Z",
  // // LimelightHelpers.getBotPose_TargetSpace("limelight")[2]);
  // // SmartDashboard.putNumber("Target Pitch",
  // // LimelightHelpers.getBotPose_TargetSpace("limelight")[3]);
  // // SmartDashboard.putNumber("Target Yaw",
  // // LimelightHelpers.getBotPose_TargetSpace("limelight")[4]);
  // // SmartDashboard.putNumber("Target Roll",
  // // LimelightHelpers.getBotPose_TargetSpace("limelight")[5]);
  // }

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
    if (Math.abs(getAngularChassisSpeed()) > Constants.Vision.kMaxRotationRate.getValue()) {
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
        OdometryThread.getFieldCorrectedAngle(),
        m_modulePositions,
        new Pose2d(0, 0, OdometryThread.getFieldCorrectedAngle()));
  }

  public double getAngularChassisSpeed() {
    return swerveSubsystem.getGyro().getAngularVelocityZWorld().getValueAsDouble();
  }

  public void setPosition(Pose2d pos) {

    SmartDashboard.putNumber("Pathplanner Angle", pos.getRotation().getRadians());
    SmartDashboard.putNumber("Setting Angle", OdometryThread.getFieldCorrectedAngle().getRadians());

    updateSwerveModulePositions();
    poseEstimator.resetPosition(
        OdometryThread.getFieldCorrectedAngle(),
        m_modulePositions,
        pos);
  }

  public void resetGyro() {
    gyroZero = swerveSubsystem.getGyro().getRotation2d().plus(Rotation2d.fromDegrees(180.0)).getRadians();
  }

  public Command resetGyroCommand() {
    return Commands.runOnce(
        () -> {
          resetGyro();
        });
  }

  /*
   * Makes the signals updates much faster, increasing from 50hz to 250hz
   */
  public class OdometryThread extends Thread {
    private BaseStatusSignal[] m_allSignals;
    public int SuccessfulDaqs = 0;
    public int FailedDaqs = 0;

    private LinearFilter lowpass = LinearFilter.movingAverage(50);
    private double lastTime = 0;
    private double currentTime = 0;
    private double averageLoopTime = 0;
    private double ModuleCount = 4;
    public static double currentAngle;

    public OdometryThread() {
      super();
      // 4 signals for each module + 2 for Pigeon2
      m_allSignals = new BaseStatusSignal[(int) ((ModuleCount * 4) + 2)];
      for (int i = 0; i < ModuleCount; ++i) {
        var signals = swerveSubsystem.getSwerveModule(i).getSignals();
        m_allSignals[(i * 4) + 0] = signals[0];
        m_allSignals[(i * 4) + 1] = signals[1];
        m_allSignals[(i * 4) + 2] = signals[2];
        m_allSignals[(i * 4) + 3] = signals[3];
      }
      m_allSignals[m_allSignals.length - 2] = swerveSubsystem.getGyro().getYaw();
      m_allSignals[m_allSignals.length - 1] = swerveSubsystem.getGyro().getAngularVelocityZWorld();
    }

    @Override
    public void run() {
      /* Make sure all signals update at around 250hz */
      for (var sig : m_allSignals) {
        sig.setUpdateFrequency(250);
      }
      /* Run as fast as possible, our signals will control the timing */
      while (true) {
        /* Synchronously wait for all signals in drivetrain */
        var status = BaseStatusSignal.waitForAll(0.1, m_allSignals);
        lastTime = currentTime;
        currentTime = Utils.getCurrentTimeSeconds();
        averageLoopTime = lowpass.calculate(currentTime - lastTime);

        /* Get status of the waitForAll */
        if (status.isOK()) {
          SuccessfulDaqs++;
        } else {
          FailedDaqs++;
        }

        /* Now update odometry */
        for (int i = 0; i < ModuleCount; ++i) {
          /*
           * No need to refresh since it's automatically refreshed from the waitForAll()
           */
          m_modulePositions[i] = m_modules[i].getPosition(false);
        }
        // Assume Pigeon2 is flat-and-level so latency compensation can be performed
        Measure<AngleUnit> yawDegrees = BaseStatusSignal.getLatencyCompensatedValue(
            swerveSubsystem.getGyro().getYaw(), swerveSubsystem.getGyro().getAngularVelocityZWorld());
        double yawDegreesRadians = yawDegrees.in(Units.Radians);

        updateCurrentAngle(yawDegreesRadians);

        /*
         * Update the swerve modules now (accounting for Limelight)
         */
        poseEstimator.update(
            OdometryThread.getFieldCorrectedAngle(),
            m_modulePositions);

        addLLVisionMeasurement();

        m_field.setRobotPose(m_odometry.getPoseMeters());
      }
    }

    public SwerveModulePosition[] getModulePositions() {
      return m_modulePositions;
    }

    /*
     * gives the actual angle that we need, similar to gyroZero
     */
    public static void updateCurrentAngle(double yawDegreesRadians) {
      currentAngle = yawDegreesRadians + Math.toRadians(180) - gyroZero;
    }

    /**
     * Accounts for where foward is for swerve pos estimation (red/blue alliance)
     */
    public static Rotation2d getFieldCorrectedAngle() {
      if (swerveSubsystem.isBlueAlliance()) {
        return new Rotation2d(currentAngle);
      } else {
        return new Rotation2d(currentAngle + Math.PI);
      }
    }
  }
}
