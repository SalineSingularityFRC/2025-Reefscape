// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.Map;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.wpilibj2.command.Commands.defer;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.select;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lib.pose.GeneralPose;
import lib.pose.ScoreConfig.TargetObject;
import lib.pose.ScoreConfig.TargetState;
import lib.vision.Limelight;
import lib.vision.RealSenseCamera;
import frc.robot.commands.ButtonDriveController;
import frc.robot.commands.DriveController;
import frc.robot.commands.RumbleCommandStart;
import frc.robot.commands.RumbleCommandStop;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Setpoint;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.LEDStatusSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    private SwerveSubsystem swerveSubsystem;
    private CommandXboxController driverController;
    private CommandXboxController operatorController;
    private CommandXboxController thirdController;
    private SendableChooser<Command> autoChooser;
    private CoralSubsystem coralSubsystem;
    // private CommandGenericHID simController;
    private ElevatorSubsystem elevatorSubsystem;
    private Limelight leftLimelight;
    private Limelight rightLimelight;
    private RealSenseCamera realSenseCamera;
    private LEDStatusSubsystem ledStatusSubsystem;
    private AlgaeSubsystem algaeSubsystem;

    protected RobotContainer() {
        coralSubsystem = new CoralSubsystem();
        elevatorSubsystem = new ElevatorSubsystem(coralSubsystem);
        leftLimelight = new Limelight(Constants.Vision.Names.leftLL);
        rightLimelight = new Limelight(Constants.Vision.Names.rightLL);
        realSenseCamera = new RealSenseCamera(Constants.Vision.Names.realSenseCam);
        swerveSubsystem = new SwerveSubsystem(leftLimelight, rightLimelight);
        // climber = new ClimberSubsystem();
        ledStatusSubsystem = new LEDStatusSubsystem(coralSubsystem, elevatorSubsystem);
        // trough = new TroughSubsystem();
        algaeSubsystem = new AlgaeSubsystem();

        driverController = new CommandXboxController(Constants.Gamepad.Controller.DRIVE);
        operatorController = new CommandXboxController(Constants.Gamepad.Controller.BUTTON);
        thirdController = new CommandXboxController(Constants.Gamepad.Controller.THIRD);

        configureBindings();

        NamedCommands.registerCommand("Stop Driving", swerveSubsystem.stopDriving());
        NamedCommands.registerCommand("Feeder Station", elevatorSubsystem.targetPosition(Setpoint.kFeederStation));
        NamedCommands.registerCommand("L4", elevatorSubsystem.targetPosition(Setpoint.kLevel4));
        NamedCommands.registerCommand("L2", elevatorSubsystem.targetPosition(Setpoint.kLevel2));
        NamedCommands.registerCommand("L3", elevatorSubsystem.targetPosition(Setpoint.kLevel3));
        NamedCommands.registerCommand("Pre-L4", elevatorSubsystem.autonTargetPosition(Setpoint.kLevel4));

        NamedCommands.registerCommand("Intake Coral", coralSubsystem.intakeCoral());
        NamedCommands.registerCommand("Shoot Coral", coralSubsystem.shootCoral().alongWith(buildCoralAssistCommand()));
        NamedCommands.registerCommand("Wait For Coral", coralSubsystem.waitUntilCoral());

        NamedCommands.registerCommand("Move Hinge Coral", algaeSubsystem.moveToCoralScorePose());
        NamedCommands.registerCommand("Move Hinge Zero", algaeSubsystem.moveToZero());
        NamedCommands.registerCommand("Move To Algae Intake", algaeSubsystem.moveToIntakePos());
        NamedCommands.registerCommand("Move Hinge Barge", algaeSubsystem.moveToAlgaeShoot());
        NamedCommands.registerCommand("Auto Barge Score", buildBargeScoringRoutine());

        NamedCommands.registerCommand("Intake Algae", algaeSubsystem.intake());
        NamedCommands.registerCommand("Shoot ALgae", algaeSubsystem.shootAlgae());

        NamedCommands.registerCommand("RumbleCommantStart", new RumbleCommandStart(driverController));
        NamedCommands.registerCommand("RumbleCommantStop", new RumbleCommandStop(driverController));

        // For convenience a programmer could change this when going to competition.
        final boolean filterCompetitionAutos = true;

        // Build an auto chooser. This will use Commands.none() as the default option.
        // As an example, this will only show autos that start with "comp" while at
        // competition as defined by the programmer
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
                (stream) -> filterCompetitionAutos
                        ? stream.filter(
                                auto -> (auto.getName().startsWith("Kick") || auto.getName().startsWith("Center")))
                        : stream);

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Redundent elevator positions of drive controller
        driverController.a().onTrue(elevatorSubsystem.moveToTargetPosition(Setpoint.kLevel1).withName("kLevel1"));
        driverController.b().onTrue(elevatorSubsystem.moveToTargetPosition(Setpoint.kLevel2).withName("kLevel2"));
        driverController.x().onTrue(elevatorSubsystem.moveToTargetPosition(Setpoint.kLevel3).withName("kLevel3"));
        driverController.y().onTrue(elevatorSubsystem.moveToTargetPosition(Setpoint.kLevel4).withName("kLevel4"));

        // Redundent manual elevator movement and coral box intaking/shooting
        driverController.povDown().whileTrue(elevatorSubsystem.runMotors(true).withName("runMotorsReverseTrue"));
        driverController.povUp().whileTrue(elevatorSubsystem.runMotors(false).withName("runMotorsReverseFalse"));
        driverController.povLeft().whileTrue(coralSubsystem.intakeCoral().withName("intakeCoral"));
        driverController.povRight().whileTrue(coralSubsystem.shootCoral().withName("shootCoral"));

        // Doesn't work since CameraDriveToPose PIDs to a field centric pose
        // Need to rewrite CameraDriveToPose to be robot centric
        // driveController.rightTrigger().whileTrue(drive.cameraDriveToPose(cam));

        // Reset gryo
        driverController.rightBumper().onTrue(swerveSubsystem.resetGyroCommand());

        // Algae controls
        driverController.leftTrigger().whileTrue(buildAlgaeIntakeRoutine());
        driverController.leftBumper().whileTrue(algaeSubsystem.moveToZero().withName("returnToHomePosAlgae"));
        driverController.rightTrigger().whileTrue(algaeSubsystem.shootAlgae().withName("shootAlgae"));
        driverController.rightTrigger().onFalse(algaeSubsystem.hold(0));

        thirdController.povUp().whileTrue(algaeSubsystem.manualControlForward());
        thirdController.povUp().onFalse(algaeSubsystem.mainMotorHoldCommand());
        thirdController.povDown().whileTrue(algaeSubsystem.manualControlBackwards());
        thirdController.povUp().onFalse(algaeSubsystem.mainMotorHoldCommand());

        thirdController.a().whileTrue(algaeSubsystem.moveToAlgaeShoot());
        thirdController.b().whileTrue(algaeSubsystem.manualIntake());
        thirdController.y().whileTrue(coralSubsystem.shootL1Coral());

        thirdController.x().whileTrue(buildAutoRoutine(TargetState.ALGAE_BUTTON));

        // PID to nearest coral pose left and score into barge
        operatorController.a().whileTrue(buildBargeScoringRoutine());
        operatorController.b().whileTrue(buildAutoRoutine(TargetState.L2_LEFT));
        operatorController.x().whileTrue(buildAutoRoutine(TargetState.L3_LEFT));
        operatorController.y().whileTrue(buildAutoRoutine(TargetState.L4_LEFT));

        // PID to nearest coral pose right
        operatorController.leftBumper()
                .onTrue(elevatorSubsystem.moveToTargetPosition(Setpoint.kFeederStation).withName("kFeederStation"));
        operatorController.rightBumper().whileTrue(buildAutoRoutine(TargetState.L2_RIGHT));
        operatorController.back().whileTrue(buildAutoRoutine(TargetState.L3_RIGHT));
        operatorController.start().whileTrue(buildAutoRoutine(TargetState.L4_RIGHT));

        // PID to coral source
        operatorController.button(11).whileTrue(buildAutoRoutine(TargetState.L1_LEFT));
        operatorController.button(12).whileTrue(buildAutoRoutine(TargetState.L1_RIGHT));

        // Intaking and shooting coral logic
        operatorController.rightStick().whileTrue(coralSubsystem.intakeCoral().withName("intakeCoral"));
        operatorController.leftStick()
                .whileTrue(coralSubsystem.shootCoral().withName("shootCoral").alongWith(buildCoralAssistCommand()));
        // buttonController.leftStick().whileTrue(algae.moveToCoralScorePose().withName("Algae
        // Hinge to Coral Pose"));
        operatorController.leftStick().whileFalse(algaeSubsystem.moveToZero());

        // driveController.povRight().onTrue(drive.xMode());

        // Driving
        swerveSubsystem.setDefaultCommand(
                new DriveController(swerveSubsystem, () -> {
                    return driverController.getRightX();
                }, () -> {
                    return driverController.getLeftY();
                }, () -> {
                    return driverController.getLeftX();
                },
                        Constants.SwerveModule.Speed.MAX_SPEED));

        // Fine adjustments for coral scroing
        operatorController.axisGreaterThan(0, 0.1).whileTrue(
                new ButtonDriveController(swerveSubsystem, () -> {
                    return 0.0;
                }, () -> {
                    return 0.0;
                }, () -> {
                    return 1.0;
                },
                        0.2));
        operatorController.axisLessThan(0, -0.1).whileTrue(
                new ButtonDriveController(swerveSubsystem, () -> {
                    return 0;
                }, () -> {
                    return 0;
                }, () -> {
                    return -1;
                },
                        0.2));

        // Manual elevator movement
        operatorController.axisGreaterThan(1, 0.1).whileTrue(elevatorSubsystem.runMotors(true)
                .withName("runMotorsReverseTrue"));

        operatorController.axisLessThan(1, -0.1).whileTrue(elevatorSubsystem.runMotors(false)
                .withName("runMotorsReverseFalse"));

    }

    protected Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    protected void updateOdometry() {
        this.swerveSubsystem.updateOdometry();
    }

    protected void updateCamera() {
        realSenseCamera.updateReefPose();
    }

    protected void updateMatchTime() {
        SmartDashboard.putNumber("Elastic/Match Time", DriverStation.getMatchTime());
    }

    // protected void updateCamData() {
    // this.cam.updateReefPose();
    // }

    protected void zeroRotation() {
        this.swerveSubsystem.resetGyro();
    }

    protected void updateRotationPIDSetpoint() {
        this.swerveSubsystem.updateRotationPIDSetpoint();
    }

    /**
     * Creates the full autonomous command sequence for the specified TargetState.
     *
     * <p>
     * This method defers construction so that
     * {@code drive.filterClosestState(target)}
     * is called when the command is scheduled to calculate the closest was Pose2d.
     *
     * <p>
     * Once initialized, it runs the drivetrain and elevator in parallel to move to
     * the computed pose, selecting between coral or algae-specific routines.
     * After the chosen routine completes, it stops the drivetrain.
     *
     * <p>
     * Requirements: {@code DriveSubsystem} and {@code ElevatorSubsystem}.
     *
     * @param target The TargetState to reach
     * @return A deferred, composable Command that handles pose filtering, parallel
     *         motion and branch-specific actions.
     */
    public Command buildAutoRoutine(TargetState target) {
        // Defer the entire routine so filterClosestState() is only called once
        return defer(() -> {
            GeneralPose generalPose = swerveSubsystem.filterClosestState(target);

            // Base parallel: drive + elevator in parallel
            Command base = parallel(
                    swerveSubsystem.driveToPose(generalPose.getPose2d()),
                    elevatorSubsystem.moveToTargetPosition(generalPose.getTargetState().getSetpoint()),
                    swerveSubsystem.updateRotationPIDSetpointCommand());

            // Two branches: coral vs. algae
            Command coralFlow = base;
            Command algaeFlow = base.alongWith(buildAlgaeIntakeRoutine())
                    .andThen(swerveSubsystem.backAwayFromReef());

            // Select based on TargetObject, then stop drive once done
            return select(
                    Map.of(
                            TargetObject.ALGAE, algaeFlow,
                            TargetObject.CORAL, coralFlow),
                    () -> target.getObject())
                    .andThen(Commands.runOnce(swerveSubsystem::stopDriving, swerveSubsystem));
        },
                // Declare requirements up front
                Set.of(swerveSubsystem, elevatorSubsystem));
    }

    // private Command makeL4AutoScoreCommand(TargetState target, RealSenseCamera
    // camera) {
    // Command driveToReef = drive.cameraDriveToPose(camera,
    // target).andThen(drive.updateRotationPIDSetpointCommand());
    // ParallelCommandGroup commandGroup = new ParallelCommandGroup();
    // commandGroup.addCommands(driveToReef);
    // commandGroup.addCommands(elevator.moveToTargetPosition(targetToSetPoint(target)));
    // return commandGroup.andThen(drive.stopDriving());
    // }

    // Drive + elevator + algae + barge scoring in one sequence:
    private Command buildBargeScoringRoutine() {
        return Commands.sequence(
                // 1) Drive close to barge in parallel with elevator and algae pre-move
                Commands.parallel(
                        swerveSubsystem.driveCloseToBargePose()
                                .andThen(swerveSubsystem.stopDriving())
                                .andThen(swerveSubsystem.updateRotationPIDSetpointCommand()),
                        algaeSubsystem.moveToAlgaeShoot(),
                        elevatorSubsystem.moveToTargetPosition(Setpoint.kLevel2)),
                // 2) Ramp elevator to L4, pause, then actually drive into barge
                elevatorSubsystem.moveToTargetPosition(Setpoint.kLevel4),
                new WaitCommand(1),
                swerveSubsystem.driveToBargePose()
                        .andThen(swerveSubsystem.stopDriving())
                        .andThen(swerveSubsystem.updateRotationPIDSetpointCommand()),
                new WaitCommand(0.5),
                // 3) Finally shoot algae
                algaeSubsystem.shootAlgae());
    }

    // Intake → delay → reposition in one sequence
    private Command buildAlgaeIntakeRoutine() {
        return Commands.sequence(
                algaeSubsystem.intake(),
                new WaitCommand(1),
                algaeSubsystem.moveToAlgaeShoot());
    }

    // Conditional “assist” for coral scoring
    private Command buildCoralAssistCommand() {
        return new ConditionalCommand(
                // if elevator at L4, run a deadline group to raise & intake
                new ParallelDeadlineGroup(
                        elevatorSubsystem.moveL4RaisedSlow(Setpoint.kLevel4Raised),
                        algaeSubsystem.moveToCoralScorePose().andThen(algaeSubsystem.runMotorsToIntake())),
                // else do nothing
                new InstantCommand(),
                elevatorSubsystem::isElevatorAtL4);
    }

}
