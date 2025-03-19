// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.controller.DifferentialDriveAccelerationLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lib.vision.Limelight;
import lib.vision.RealSenseCamera;
import frc.robot.commands.ButtonDriveController;
import frc.robot.commands.CameraDriveToPose;
import frc.robot.commands.DriveController;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.RumbleCommandStart;
import frc.robot.commands.RumbleCommandStop;
import frc.robot.subsystems.AlgaeSubsystem;
// import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Setpoint;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDStatusSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.AutoScoreTarget;
import frc.robot.subsystems.TroughSubsystem;

public class RobotContainer {
    private SwerveSubsystem drive;
    private CommandXboxController driveController;
    private CommandXboxController buttonController;
    private CommandXboxController thirdController;
    private SendableChooser<Command> autoChooser;
    private IntakeSubsystem intake;
    // private CommandGenericHID simController;
    private ElevatorSubsystem elevator;
    private Limelight leftLL;
    private Limelight rightLL;
    // private RealSenseCamera cam;
    private LEDStatusSubsystem ledStatus;
    private AlgaeSubsystem algae;

    protected RobotContainer() {
        intake = new IntakeSubsystem();
        elevator = new ElevatorSubsystem(intake);
        leftLL = new Limelight(Constants.Vision.Names.leftLL);
        rightLL = new Limelight(Constants.Vision.Names.rightLL);
        // cam = new RealSenseCamera(Constants.Vision.Names.realSenseCam);
        drive = new SwerveSubsystem(leftLL, rightLL);
        // climber = new ClimberSubsystem();
        ledStatus = new LEDStatusSubsystem(intake, elevator);
        // trough = new TroughSubsystem();
        algae = new AlgaeSubsystem();

        driveController = new CommandXboxController(Constants.Gamepad.Controller.DRIVE);
        buttonController = new CommandXboxController(Constants.Gamepad.Controller.BUTTON);
        thirdController = new CommandXboxController(Constants.Gamepad.Controller.THIRD);

        configureBindings();

        NamedCommands.registerCommand("Stop Driving", drive.stopDriving());
        NamedCommands.registerCommand("Feeder Station", elevator.targetPosition(Setpoint.kFeederStation));
        NamedCommands.registerCommand("L4", elevator.targetPosition(Setpoint.kLevel4));
        NamedCommands.registerCommand("L2", elevator.targetPosition(Setpoint.kLevel2));
        NamedCommands.registerCommand("Pre-L4", elevator.autonTargetPosition(Setpoint.kLevel4));
        NamedCommands.registerCommand("Intake Coral", intake.intakeCoral());
        NamedCommands.registerCommand("Shoot Coral", intake.shootCoral());
        NamedCommands.registerCommand("Wait For Coral", intake.waitUntilCoral());
        NamedCommands.registerCommand("RumbleCommantStart", new RumbleCommandStart(driveController));
        NamedCommands.registerCommand("RumbleCommantStop", new RumbleCommandStop(driveController));

        // For convenience a programmer could change this when going to competition.
        boolean isCompetition = false;

        // Build an auto chooser. This will use Commands.none() as the default option.
        // As an example, this will only show autos that start with "comp" while at
        // competition as defined by the programmer
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
                (stream) -> isCompetition
                        ? stream.filter(auto -> (auto.getName().startsWith("Bottom") || auto.getName().startsWith("Top")
                                || auto.getName().startsWith("Center")))
                        : stream);

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void initialize() {
        drive.initialize();
    }

    private void configureBindings() {
        // Redundent elevator positions of drive controller
        driveController.a().onTrue(elevator.moveToTargetPosition(Setpoint.kLevel1).withName("kLevel1"));
        driveController.b().onTrue(elevator.moveToTargetPosition(Setpoint.kLevel2).withName("kLevel2"));
        driveController.x().onTrue(elevator.moveToTargetPosition(Setpoint.kLevel3).withName("kLevel3"));
        driveController.y().onTrue(elevator.moveToTargetPosition(Setpoint.kLevel4).withName("kLevel4"));

        // Redundent manual elevator movement and coral box intaking/shooting
        driveController.povDown().whileTrue(elevator.runMotors(true).withName("runMotorsReverseTrue"));
        driveController.povUp().whileTrue(elevator.runMotors(false).withName("runMotorsReverseFalse"));
        driveController.povLeft().whileTrue(intake.intakeCoral().withName("intakeCoral"));
        driveController.povRight().whileTrue(intake.shootCoral().withName("shootCoral"));

        // driveController.leftTrigger().whileTrue(new DeferredCommand(() -> {
        //     return new DriveToPose(drive, drive.supplier_position, () -> {
        //         return new Pose2d(16, 4, Rotation2d.fromDegrees(180));
        //     });
        // }, Set.of(drive)));

        // Doesn't work since CameraDriveToPose PIDs to a field centric pose
        // Need to rewrite CameraDriveToPose to be robot centric
        // driveController.rightTrigger().whileTrue(drive.cameraDriveToPose(cam));

        // driveController.rightBumper().whileTrue(trough.moveTroughForward());
        // driveController.leftBumper().whileTrue(trough.moveTroughBack());

        driveController.rightBumper().onTrue(drive.resetGyroCommand()); //TEMPORARY CHANGE LATER

        // TEMPORARY ALGAE COMMAND BUTTON STUFF \\
        driveController.leftTrigger().whileTrue(algae.intake().withName("intakeAlgae"));
        driveController.leftTrigger().onFalse(algae.hold(3));
        // driveController.rightBumper().onTrue(algae.moveToIntakePos().withName("movetointakepos"));
        driveController.leftBumper().onTrue(algae.returnToHomePos().withName("returnToHomePosAlgae"));
        driveController.rightTrigger().whileTrue(algae.shootAlgae().withName("shootAlgae"));
        driveController.rightTrigger().onFalse(algae.hold(0));

        thirdController.povUp().whileTrue(algae.manualControlForward());
        thirdController.povUp().onFalse(algae.mainMotorHoldCommand());
        thirdController.povDown().whileTrue(algae.manualControlBackwards());
        thirdController.povUp().onFalse(algae.mainMotorHoldCommand());

        buttonController.a().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L1_LEFT));
        buttonController.b().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L2_LEFT));
        buttonController.x().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L3_LEFT));
        buttonController.y().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L4_LEFT));
        buttonController.button(11).whileTrue(makeAutoDriveToSourceCommand(AutoScoreTarget.L1_LEFT));
        buttonController.button(12).whileTrue(makeAutoDriveToSourceCommand(AutoScoreTarget.L1_RIGHT));

        buttonController.leftBumper().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L1_RIGHT));
        buttonController.rightBumper().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L2_RIGHT));
        buttonController.back().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L3_RIGHT));
        buttonController.start().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L4_RIGHT));

        // buttonController.rightStick().whileTrue(trough.moveToZeroPostion());
        // buttonController.leftStick().whileTrue(trough.moveToClimbPositon());
        buttonController.rightStick().whileTrue(intake.intakeCoral().withName("intakeCoral"));
        buttonController.leftStick().whileTrue(intake.shootCoral().withName("shootCoral"));
        buttonController.leftStick().whileTrue(algae.moveToCoralScorePose());
        buttonController.leftStick().onFalse(algae.moveToZero());

        // driveController.povDown().whileTrue(intake.runMotorsBack());
        // driveController.leftBumper().whileTrue(intake.intakeCoral());
        // driveController.rightBumper().whileTrue(intake.shootCoral());

        // elevatorController.button(1).whileTrue(elevator.moveToTargetPosition(Setpoint.kLevel1));
        // // Red
        // elevatorController.button(2).whileTrue(elevator.moveToTargetPosition(Setpoint.kLevel2));
        // // Blue
        // elevatorController.button(3).whileTrue(elevator.moveToTargetPosition(Setpoint.kLevel3));
        // // Yellow

        // driveController.povUp().whileTrue(
        // new DriveController(drive, () -> {
        // return 0;
        // }, () -> {
        // return 1;
        // }, () -> {
        // return 0;
        // },
        // 2));

        // driveController.povDown().whileTrue(
        // new DriveController(drive, () -> {
        // return 0;
        // }, () -> {
        // return -1;
        // }, () -> {
        // return 0;
        // },
        // 2));

        // driveController.povRight().onTrue(drive.xMode());

        // driveController.povRight().onTrue(drive.xMode());

        drive.setDefaultCommand(
                new DriveController(drive, () -> {
                    return driveController.getRightX();
                }, () -> {
                    return driveController.getLeftY();
                }, () -> {
                    return driveController.getLeftX();
                },
                        Constants.SwerveModule.Speed.MAX_SPEED));

        buttonController.axisGreaterThan(0, 0.1).whileTrue(
                new ButtonDriveController(drive, () -> {
                    return 0.0;
                }, () -> {
                    return 0.0;
                }, () -> {
                    return 1.0;
                },
                        0.2));

        buttonController.axisLessThan(0, -0.1).whileTrue(
                new ButtonDriveController(drive, () -> {
                    return 0;
                }, () -> {
                    return 0;
                }, () -> {
                    return -1;
                },
                        0.2));

        buttonController.axisGreaterThan(1, 0.1).whileTrue(elevator.runMotors(true)
                .withName("runMotorsReverseTrue"));

        buttonController.axisLessThan(1, -0.1).whileTrue(elevator.runMotors(false)
                .withName("runMotorsReverseFalse"));

    }

    protected Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    protected void updateOdometry() {
        this.drive.updateOdometry();
    }

    protected void updateMatchTime() {
        SmartDashboard.putNumber("Elastic/Match Time", DriverStation.getMatchTime());
    }

    // protected void updateCamData() {
    //     this.cam.updateReefPose();
    // }

    protected void zeroRotation() {
        this.drive.resetGyro();
    }

    protected void updateRotationPIDSetpoint() {
        this.drive.updateRotationPIDSetpoint();
    }

    private Command makeAutoScoreCommand(AutoScoreTarget target) {
        ParallelCommandGroup commandGroup = new ParallelCommandGroup();
        commandGroup.addCommands(drive.drivetoReefPose(target).andThen(drive.updateRotationPIDSetpointCommand()));
        commandGroup.addCommands(elevator.moveToTargetPosition(targetToSetPoint(target)));
        return commandGroup.andThen(drive.stopDriving());
    }

    private Command makeAutoDriveToSourceCommand(AutoScoreTarget target) {
        ParallelCommandGroup commandGroup = new ParallelCommandGroup();
        commandGroup.addCommands(drive.drivetoSourcePose(target).andThen(drive.updateRotationPIDSetpointCommand()));
        commandGroup.addCommands(elevator.moveToTargetPosition(targetToSetPoint(target)));
        return commandGroup.andThen(drive.stopDriving());
    }

    private Setpoint targetToSetPoint(AutoScoreTarget target) {
        switch (target) {
            case L4_LEFT:
            case L4_RIGHT:
                return Setpoint.kLevel4;
            case L3_LEFT:
            case L3_RIGHT:
                return Setpoint.kLevel3;
            case L2_LEFT:
            case L2_RIGHT:
                return Setpoint.kLevel2;
            case L1_LEFT:
            case L1_RIGHT:
                return Setpoint.kLevel1;
            default:
                return Setpoint.kFeederStation;
        }
    }

}
