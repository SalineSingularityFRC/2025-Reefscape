// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.DriveController;
import frc.robot.commands.RumbleCommandStart;
import frc.robot.commands.RumbleCommandStop;
import frc.robot.commands.toSpeaker;

public class RobotContainer {
    private SwerveSubsystem drive;
    private Limelight lime;
    private CommandXboxController driveController;
    private SendableChooser<String> pathAutonChooser;

    protected RobotContainer() {
        lime = new Limelight();
        drive = new SwerveSubsystem();

        driveController = new CommandXboxController(Constants.Gamepad.Controller.DRIVE);

        configureBindings();

        NamedCommands.registerCommand("StopDriving", drive.stopDriving());
        NamedCommands.registerCommand("RumbleCommantStart", new RumbleCommandStart(driveController));
        NamedCommands.registerCommand("RumbleCommantStop", new RumbleCommandStop(driveController));

        this.pathAutonChooser = new SendableChooser<String>();

        this.pathAutonChooser.setDefaultOption("Fake Auto", "Fake Auto");
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
        driveController.x().whileTrue(drive.resetGyroCommand());

        // driveController.b().whileTrue(
        //         new toSpeaker(drive, lime)
        // );

        driveController.a().whileTrue(
            new DriveController(drive, () -> {
                return 0;
            }, () -> {
                return 1;
            }, () -> {
                return 0;
            },
                2));

        driveController.b().whileTrue(
            new DriveController(drive, () -> {
                return 0;
            }, () -> {
                return -1;
            }, () -> {
                return 0;
            },
                2));

        driveController.povRight().onTrue(drive.xMode());

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
                        4.0));
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

}
