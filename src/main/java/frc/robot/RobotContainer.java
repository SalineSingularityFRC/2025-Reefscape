// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Setpoint;
import frc.robot.commands.DriveController;
import frc.robot.commands.RumbleCommandStart;
import frc.robot.commands.RumbleCommandStop;
import frc.robot.commands.toSpeaker;

public class RobotContainer {
    private SwerveSubsystem drive;
    private Limelight lime;
    private CommandXboxController driveController;
    private SendableChooser<String> pathAutonChooser;
    private IntakeSubsystem intake;
    // private CommandGenericHID simController;
    private ElevatorSubsystem elevator;

    protected RobotContainer() {
        lime = new Limelight();
        drive = new SwerveSubsystem();
        intake = new IntakeSubsystem();
        elevator = new ElevatorSubsystem();

        driveController = new CommandXboxController(Constants.Gamepad.Controller.DRIVE);

        configureBindings();

        NamedCommands.registerCommand("StopDriving", drive.stopDriving());
        NamedCommands.registerCommand("RumbleCommantStart", new RumbleCommandStart(driveController));
        NamedCommands.registerCommand("RumbleCommantStop", new RumbleCommandStop(driveController));

        this.pathAutonChooser = new SendableChooser<String>();

        this.pathAutonChooser.setDefaultOption("Noah's Auto", "New Auto");
        this.pathAutonChooser.setDefaultOption("posEstimator Test", "Short Auto");
        SmartDashboard.putData("Auton Choices", pathAutonChooser);
    }

    private void configureBindings() {
        driveController.a().whileTrue(intake.runMotors());

        //driveController.b().whileTrue(elevator.moveToTargetPosition(Setpoint.kFeederStation));
        //driveController.y().whileTrue(elevator.moveToTargetPosition(Setpoint.kLevel4));

        driveController.b().whileTrue(elevator.runMotors(true));
        driveController.y().whileTrue(elevator.runMotors(false));

        driveController.x().whileTrue(drive.resetGyroCommand());

        driveController.a().whileTrue(intake.runMotors());

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
                        1.0));
    }

    protected Command getAutonomousCommand() {
        return new PathPlannerAuto(this.pathAutonChooser.getSelected());
    }

    protected void updateOdometry() {
        this.drive.updateOdometry();
    }

}
