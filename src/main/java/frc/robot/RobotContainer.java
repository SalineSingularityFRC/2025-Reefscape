// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ButtonDriveController;
import frc.robot.commands.DriveController;
import frc.robot.commands.RumbleCommandStart;
import frc.robot.commands.RumbleCommandStop;
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
    private Limelight lime;
    private CommandXboxController driveController;
    private CommandXboxController buttonController;
    private SendableChooser<Command> autoChooser;
    private IntakeSubsystem intake;
    // private CommandGenericHID simController;
    private ElevatorSubsystem elevator;
    private ClimberSubsystem climber;
    private TroughSubsystem trough;
    private LEDStatusSubsystem ledStatus;

    protected RobotContainer() {
        lime = new Limelight();
        drive = new SwerveSubsystem();
        intake = new IntakeSubsystem();
        elevator = new ElevatorSubsystem(intake);
        climber = new ClimberSubsystem();
        ledStatus = new LEDStatusSubsystem(intake, lime, elevator);
        trough = new TroughSubsystem();

        driveController = new CommandXboxController(Constants.Gamepad.Controller.DRIVE);
        buttonController = new CommandXboxController(Constants.Gamepad.Controller.BUTTON);

        configureBindings();

        NamedCommands.registerCommand("Stop Driving", drive.stopDriving());
        NamedCommands.registerCommand("Feeder Station", elevator.targetPosition(Setpoint.kFeederStation));
        NamedCommands.registerCommand("L4", elevator.targetPosition(Setpoint.kLevel4));
        NamedCommands.registerCommand("Intake Coral", intake.intakeCoral());
        NamedCommands.registerCommand("Shoot Coral", intake.shootCoral());
        NamedCommands.registerCommand("RumbleCommantStart", new RumbleCommandStart(driveController));
        NamedCommands.registerCommand("RumbleCommantStop", new RumbleCommandStop(driveController));

        // For convenience a programmer could change this when going to competition.
        boolean isCompetition = true;

        // Build an auto chooser. This will use Commands.none() as the default option.
        // As an example, this will only show autos that start with "comp" while at
        // competition as defined by the programmer
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
                (stream) -> isCompetition
                        ? stream.filter(auto -> (auto.getName().startsWith("Bottom") || auto.getName().startsWith("Top")
                                || auto.getName().startsWith("Center")))
                        : stream);

        // this.autoChooser.setDefaultOption("Center/ H - A", "Center - H - A");
        // this.autoChooser.addOption("Center/ H - B", "Center - H - B");
        // this.autoChooser.addOption("Center/ H - I", "Center - H - I");
        // this.autoChooser.addOption("Center/ H - J", "Center - H - J");
        // this.autoChooser.addOption("Center/ H - K", "Center - H - K");
        // this.autoChooser.addOption("Center/ H - L", "Center - H - L");
        // this.autoChooser.addOption("Top/ K - A", "TopTop - K - A");
        // this.autoChooser.addOption("Top/ K - L", "TopTop - K - L");
        // this.autoChooser.addOption("Top/ J - B", "Top - J - B");
        // this.autoChooser.addOption("Top/ J - I", "Top - J - I");
        // this.autoChooser.addOption("Top/ J - L - K", "Top - J - L - K");
        // this.autoChooser.addOption("Top/ J - K - L", "Top - J - K - L");
        // this.autoChooser.addOption("Bottom/ E - C - D", "Bottom - E - C - D");
        // this.autoChooser.addOption("Bottom/ G - F", "Bottom - G - F");
        // this.autoChooser.addOption("Bottom/ E - B", "Bottom - E - B");
        // this.autoChooser.addOption("Bottom/ G - C", "Bottom - G - C");
        // this.autoChooser.addOption("Bottom/ G - B", "Bottom - G - B");

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void initialize() {
        drive.initialize();
    }

    private void configureBindings() {
        // Elevator Position
        driveController.a().onTrue(elevator.moveToTargetPosition(Setpoint.kLevel1).withName("kLevel1"));
        driveController.b().onTrue(elevator.moveToTargetPosition(Setpoint.kLevel2).withName("kLevel2"));
        driveController.x().onTrue(elevator.moveToTargetPosition(Setpoint.kLevel3).withName("kLevel3"));
        driveController.y().onTrue(elevator.moveToTargetPosition(Setpoint.kLevel4).withName("kLevel4"));

        driveController.povDown().whileTrue(elevator.runMotors(true).withName("runMotorsReverseTrue"));
        driveController.povUp().whileTrue(elevator.runMotors(false).withName("runMotorsReverseFalse"));
        driveController.povLeft().whileTrue(intake.intakeCoral().withName("intakeCoral"));
        driveController.povRight().whileTrue(intake.shootCoral().withName("shootCoral"));
        // driveController.leftTrigger().whileTrue(climber.moveWinchForward());
        // driveController.rightTrigger().whileTrue(climber.moveWinchBack());

        // driveController.rightBumper().whileTrue(trough.moveTroughForward());
        // driveController.leftBumper().whileTrue(trough.moveTroughBack());

        driveController.rightBumper().onTrue(drive.resetGyroCommand());

        buttonController.a().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L1_LEFT));
        buttonController.b().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L2_LEFT));
        buttonController.x().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L3_LEFT));
        buttonController.y().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L4_LEFT));

        buttonController.leftBumper().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L1_RIGHT));
        buttonController.rightBumper().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L2_RIGHT));
        buttonController.back().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L3_RIGHT));
        buttonController.start().whileTrue(makeAutoScoreCommand(AutoScoreTarget.L4_RIGHT));

        // buttonController.rightStick().whileTrue(trough.moveToZeroPostion());
        // buttonController.leftStick().whileTrue(trough.moveToClimbPositon());
        buttonController.rightStick().whileTrue(intake.intakeCoral().withName("intakeCoral"));
        buttonController.leftStick().whileTrue(intake.shootCoral().withName("shootCoral"));

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

    protected void zeroRotation() {
        this.drive.resetGyro();
    }

    protected void updateRotationPIDSetpoint() {
        this.drive.updateRotationPIDSetpoint();
    }

    private Command makeAutoScoreCommand(AutoScoreTarget target) {
        ParallelCommandGroup commandGroup = new ParallelCommandGroup();
        commandGroup.addCommands(drive.testPath(target).andThen(drive.updateRotationPIDSetpointCommand()));
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
