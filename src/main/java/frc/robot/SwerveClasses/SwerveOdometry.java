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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public SwerveOdometry(SwerveSubsystem subsystem, Translation2d[] vectorKinematics) {
    this.subsystem = subsystem;

    swerveKinematics =
        new SwerveDriveKinematics(
            vectorKinematics[FL], vectorKinematics[FR], vectorKinematics[BL], vectorKinematics[BR]);

    swerveOdometry =
        new SwerveDrivePoseEstimator(
            swerveKinematics,
            new Rotation2d(subsystem.getRobotAngle()),
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
            new Pose2d(0, 0, new Rotation2d()));

    // int[] validIDs = {18};
    // LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);

  }

  public void update() {
    swerveOdometry.update(
        new Rotation2d(subsystem.getRobotAngle()),
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

    // // MegaTag 1
    // LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
    //   if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
    //   {
    //     if(mt1.rawFiducials[0].ambiguity > .7)
    //     {
    //       doRejectUpdate = true;
    //     }
    //     if(mt1.rawFiducials[0].distToCamera > 3)
    //     {
    //       doRejectUpdate = true;
    //     }
    //   }
    //   if(mt1.tagCount == 0)
    //   {
    //     doRejectUpdate = true;
    //   }

    //   if(!doRejectUpdate)
    //   {
    //     swerveOdometry.setVisionMeasurementStdDevs(VecBuilder.fill(1.5,1.5,5));
    //     swerveOdometry.addVisionMeasurement(
    //         mt1.pose,
    //         mt1.timestampSeconds);
    //   }
    
    // MegaTag 2
    LimelightHelpers.SetRobotOrientation("limelight", swerveOdometry.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if(Math.abs(subsystem.getAngularChassisSpeed()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    {
      doRejectUpdate = true;
    }
    if(mt2.tagCount == 0)
    {
      doRejectUpdate = true;
    }
    if(!doRejectUpdate)
    {
      swerveOdometry.setVisionMeasurementStdDevs(VecBuilder.fill(1.5,1.5,9999999));
      swerveOdometry.addVisionMeasurement(
          mt2.pose,
          mt2.timestampSeconds);
    }
    
    SmartDashboard.putNumber("Target X", LimelightHelpers.getBotPose_TargetSpace("limelight")[0]);
    SmartDashboard.putNumber("Target Y", LimelightHelpers.getBotPose_TargetSpace("limelight")[1]);
    SmartDashboard.putNumber("Target Z", LimelightHelpers.getBotPose_TargetSpace("limelight")[2]);
    SmartDashboard.putNumber("Target Pitch", LimelightHelpers.getBotPose_TargetSpace("limelight")[3]);
    SmartDashboard.putNumber("Target Yaw", LimelightHelpers.getBotPose_TargetSpace("limelight")[4]);
    SmartDashboard.putNumber("Target Roll", LimelightHelpers.getBotPose_TargetSpace("limelight")[5]);

  }

  public Pose2d getEstimatedPosition() {
    return swerveOdometry.getEstimatedPosition();
  }

  public void resetPosition() {
    swerveOdometry.resetPosition(
        new Rotation2d(subsystem.getRobotAngle()),
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
        new Pose2d(0, 0, new Rotation2d()));
  }

    public void setPosition(Pose2d pos) {
    swerveOdometry.resetPosition(
        new Rotation2d(subsystem.getRobotAngle()),
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
