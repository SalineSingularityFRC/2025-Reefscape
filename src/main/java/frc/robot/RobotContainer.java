// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    private SendableChooser<String> pathAutonChooser;
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

        this.pathAutonChooser = new SendableChooser<String>();

        this.pathAutonChooser.setDefaultOption("Center H A", "Center H A");
        this.pathAutonChooser.addOption("L Auto", "L Auto");
        this.pathAutonChooser.addOption("Fake Auto", "Fake Auto");
        this.pathAutonChooser.addOption("Reef", "Reef");
        this.pathAutonChooser.addOption("Modified Tag", "Modified Tag");
        this.pathAutonChooser.addOption("1 Meter Auto", "1 Meter Auto");
        this.pathAutonChooser.addOption("4 Meter Auto", "4 Meter Auto");
        SmartDashboard.putData("Auton Choices", pathAutonChooser);
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

        buttonController.rightStick().whileTrue(trough.moveToZeroPostion());
        buttonController.leftStick().whileTrue(trough.moveToClimbPositon());

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
                    if (driveController.getRightX() < 0) {
                        return -1.0 * driveController.getRightX() * driveController.getRightX();
                    }

                    return driveController.getRightX() * driveController.getRightX();
                }, () -> {
                    if (driveController.getLeftY() < 0) {
                        return -1.0 * driveController.getLeftY() * driveController.getLeftY();
                    }

                    return driveController.getLeftY() * driveController.getLeftY();
                }, () -> {
                    if (driveController.getLeftX() < 0) {
                        return -1.0 * driveController.getLeftX() * driveController.getLeftX();
                    }

                    return driveController.getLeftX() * driveController.getLeftX();
                },
                        Constants.SwerveModule.Speed.MAX_SPEED));

        buttonController.axisGreaterThan(0, 0.1).whileTrue(
                new DriveController(drive, () -> {
                    return 0;
                }, () -> {
                    return 0;
                }, () -> {
                    return 1;
                },
                        0.1));

        buttonController.axisLessThan(0, -0.1).whileTrue(
                new DriveController(drive, () -> {
                    return 0;
                }, () -> {
                    return 0;
                }, () -> {
                    return -1;
                },
                        0.1));

        buttonController.axisGreaterThan(1, 0.1).whileTrue(elevator.runMotors(true)
        .withName("runMotorsReverseTrue"));

        buttonController.axisLessThan(1, -0.1).whileTrue(elevator.runMotors(false)
        .withName("runMotorsReverseFalse"));

                
    }

    protected Command getAutonomousCommand() {
        return new PathPlannerAuto(this.pathAutonChooser.getSelected());
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
