// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.Set;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.controller.DifferentialDriveAccelerationLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lib.vision.Limelight;
import lib.vision.RealSenseCamera;
import frc.robot.commands.ButtonDriveController;
import frc.robot.commands.CameraDriveToPose;
import frc.robot.commands.DriveController;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.RumbleCommandStart;
import frc.robot.commands.RumbleCommandStop;
import frc.robot.subsystems.AlgaeProcessorSubsystem;
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
    private RealSenseCamera cam;
    private LEDStatusSubsystem ledStatus;
    private AlgaeSubsystem algae;
    private AlgaeProcessorSubsystem algaeProcessorSubsystem;

    protected RobotContainer() {
        intake = new IntakeSubsystem();
        elevator = new ElevatorSubsystem(intake);
        leftLL = new Limelight(Constants.Vision.Names.leftLL);
        rightLL = new Limelight(Constants.Vision.Names.rightLL);
        cam = new RealSenseCamera(Constants.Vision.Names.realSenseCam);
        drive = new SwerveSubsystem(leftLL, rightLL);
        // climber = new ClimberSubsystem();
        ledStatus = new LEDStatusSubsystem(intake, elevator);
        // trough = new TroughSubsystem();
        algae = new AlgaeSubsystem();
        algaeProcessorSubsystem = new AlgaeProcessorSubsystem();

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
        NamedCommands.registerCommand("Move Hinge Coral", algae.moveToCoralScorePose());
        NamedCommands.registerCommand("Move Hinge Zero", algae.moveToZero());
        NamedCommands.registerCommand("RumbleCommantStart", new RumbleCommandStart(driveController));
        NamedCommands.registerCommand("RumbleCommantStop", new RumbleCommandStop(driveController));

        // For convenience a programmer could change this when going to competition.
        boolean isCompetition = true;

        // Build an auto chooser. This will use Commands.none() as the default option.
        // As an example, this will only show autos that start with "comp" while at
        // competition as defined by the programmer
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
                (stream) -> isCompetition
                        ? stream.filter(
                                auto -> (auto.getName().startsWith("Fudge") || auto.getName().startsWith("No Kick")
                                        || auto.getName().startsWith("Bottom") || auto.getName().startsWith("Top")
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

        // Doesn't work since CameraDriveToPose PIDs to a field centric pose
        // Need to rewrite CameraDriveToPose to be robot centric
        // driveController.rightTrigger().whileTrue(drive.cameraDriveToPose(cam));

        // Reset gryo
        driveController.rightBumper().onTrue(drive.resetGyroCommand());

        // Algae controls
        driveController.leftTrigger().whileTrue(makeAlgaeIntakeCommand());
        driveController.leftBumper().whileTrue(algae.moveToZero().withName("returnToHomePosAlgae"));
        driveController.rightTrigger().whileTrue(algae.shootAlgae().withName("shootAlgae"));
        driveController.rightTrigger().onFalse(algae.hold(0));

        thirdController.povUp().whileTrue(algae.manualControlForward());
        thirdController.povUp().onFalse(algae.mainMotorHoldCommand());
        thirdController.povDown().whileTrue(algae.manualControlBackwards());
        thirdController.povUp().onFalse(algae.mainMotorHoldCommand());

        thirdController.a().onTrue(algae.moveToAlgaeShoot());
        thirdController.leftBumper().whileTrue(algaeProcessorSubsystem.intakeProcessor());
        thirdController.rightBumper().whileTrue(algaeProcessorSubsystem.spitProcessor());

        // PID to nearest coral pose left and score into barge
        buttonController.a().whileTrue(makeAutoBargeScoreCommand());
        buttonController.b().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L2_LEFT));
        buttonController.x().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L3_LEFT));
        buttonController.y().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L4_LEFT));

        // PID to nearest coral pose right
        buttonController.leftBumper()
                .onTrue(elevator.moveToTargetPosition(Setpoint.kFeederStation).withName("kFeederStation"));
        buttonController.rightBumper().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L2_RIGHT));
        buttonController.back().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L3_RIGHT));
        buttonController.start().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L4_RIGHT));

        // PID to coral source
        buttonController.button(11).whileTrue(makeAutoDriveToSourceCommand(AutoScoreTarget.L1_LEFT));
        buttonController.button(12).whileTrue(makeAutoDriveToSourceCommand(AutoScoreTarget.L1_RIGHT));

        // Intaking and shooting coral logic
        buttonController.rightStick().whileTrue(intake.intakeCoral().withName("intakeCoral"));
        buttonController.leftStick()
                .whileTrue(intake.shootCoral().withName("shootCoral").andThen(makeCoralHelpScoreCommand()));
        // buttonController.leftStick().whileTrue(algae.moveToCoralScorePose().withName("Algae
        // Hinge to Coral Pose"));
        buttonController.leftStick().whileFalse(algae.moveToZero());

        // driveController.povRight().onTrue(drive.xMode());

        // Driving
        drive.setDefaultCommand(
                new DriveController(drive, () -> {
                    return driveController.getRightX();
                }, () -> {
                    return driveController.getLeftY();
                }, () -> {
                    return driveController.getLeftX();
                },
                        Constants.SwerveModule.Speed.MAX_SPEED));

        // Fine adjustments for coral scroing
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

        // Manual elevator movement
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
    // this.cam.updateReefPose();
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

    private Command makeL4AutoScoreCommand(AutoScoreTarget target, RealSenseCamera camera) {
        Command driveToReef = drive.drivetoReefPose(target).andThen(drive.updateRotationPIDSetpointCommand());
        Command cameraDriveToPose = drive.cameraDriveToPose(cam);
        Pose2d closestReef = drive.isBlueAlliance() ? drive.getClosestReef(target)
                : FlippingUtil.flipFieldPose(drive.getClosestReef(target));
        BooleanSupplier driveToCameraSwitchSupply = () -> camera.isCameraPoseStable() && (drive.supplier_position.get()
                .getTranslation().getDistance(closestReef.getTranslation()) < 1);
        Command switchToCameraDrive = new SequentialCommandGroup(
                new WaitUntilCommand(driveToCameraSwitchSupply).deadlineFor(driveToReef),
                cameraDriveToPose);
        return new ParallelCommandGroup(
                switchToCameraDrive,
                elevator.moveToTargetPosition(targetToSetPoint(target)))
                .andThen(drive.stopDriving());
    }

    private Command makeAutoDriveToSourceCommand(AutoScoreTarget target) {
        ParallelCommandGroup commandGroup = new ParallelCommandGroup();
        commandGroup.addCommands(drive.drivetoSourcePose(target).andThen(drive.updateRotationPIDSetpointCommand()));
        commandGroup.addCommands(elevator.moveToTargetPosition(targetToSetPoint(target)));
        return commandGroup.andThen(drive.stopDriving());
    }

    private Command makeAutoBargeScoreCommand() {
        ParallelCommandGroup commandGroup = new ParallelCommandGroup();
        commandGroup.addCommands(drive.driveCloseToBargePose().andThen(drive.stopDriving())
                .andThen(drive.updateRotationPIDSetpointCommand()));
        commandGroup.addCommands(elevator.moveToTargetPosition(Setpoint.kLevel2));
        return commandGroup.andThen(elevator.moveToTargetPosition(Setpoint.kLevel4))
                .andThen(drive.driveToBargePose()).andThen(drive.stopDriving())
                .andThen(drive.updateRotationPIDSetpointCommand())
                .andThen(new WaitCommand(1)).andThen(algae.shootAlgae());
    }

    private Command makeAlgaeIntakeCommand() {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        commandGroup.addCommands(algae.intake().andThen(new WaitCommand(1)));
        commandGroup.addCommands(algae.moveToAlgaeShoot());
        return commandGroup;
    }

    private Command makeCoralHelpScoreCommand() {
        return new ConditionalCommand(coralHelpScoreCommand(true), coralHelpScoreCommand(false),
                elevator::isElevatorAtL4);
    }

    private Command coralHelpScoreCommand(boolean wantHelp) {

        if (!wantHelp) {
            return new InstantCommand();
        }

        ParallelDeadlineGroup commandGroup = new ParallelDeadlineGroup(
                elevator.moveL4RaisedSlow(Setpoint.kLevel4Raised));
        commandGroup.addCommands(algae.moveToCoralScorePose().andThen(algae.runMotorsToIntake()));

        return commandGroup;
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
