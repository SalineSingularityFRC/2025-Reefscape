// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lib.pose.GeneralPose;
import lib.pose.ScoreConfig.NavigationTarget;
import lib.pose.ScoreConfig.TargetState;
import lib.vision.Limelight;
import lib.vision.RealSenseCamera;
import frc.robot.commands.RumbleCommandStart;
import frc.robot.commands.RumbleCommandStop;
import frc.robot.commands.controller.ButtonDriveController;
import frc.robot.commands.controller.DriveController;
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

    /**
     * Constructs the RobotContainer, initializing all subsystems, controllers,
     * and binding configurations.
     */
    protected RobotContainer() {
        // Subsystem initialization
        coralSubsystem = new CoralSubsystem();
        elevatorSubsystem = new ElevatorSubsystem(coralSubsystem);
        leftLimelight = new Limelight(Constants.Vision.Names.leftLL);
        rightLimelight = new Limelight(Constants.Vision.Names.rightLL);
        realSenseCamera = new RealSenseCamera(Constants.Vision.Names.realSenseCam);
        swerveSubsystem = new SwerveSubsystem(leftLimelight, rightLimelight);
        ledStatusSubsystem = new LEDStatusSubsystem(coralSubsystem, elevatorSubsystem);
        algaeSubsystem = new AlgaeSubsystem();

        // Controller initialization
        driverController = new CommandXboxController(Constants.Gamepad.Controller.DRIVE);
        operatorController = new CommandXboxController(Constants.Gamepad.Controller.BUTTON);
        thirdController = new CommandXboxController(Constants.Gamepad.Controller.THIRD);

        // Controller binding configurations
        configureBindings();
        configureAutoBindings();
        configureElevatorBindings();
        configureCoralBindings();
        configureAlgaeBindings();
        configureDriveBindings();

        // Registering auton commands and selecting auton in Elastic
        registerNamedCommands();
        autoChooser = configureAutoChooser();
        SmartDashboard.putData("Selected Auton", autoChooser);
    }

    /**
     * Placeholder for any initial bindings not covered by specialized methods.
     */
    private void configureBindings() {
        // Doesn't work since CameraDriveToPose PIDs to a field centric pose
        // Need to rewrite CameraDriveToPose to be robot centric
        // driveController.rightTrigger().whileTrue(drive.cameraDriveToPose(cam));
    }

    /**
     * Binds autonomous routines to operator/controller inputs (Reef, Coral Source,
     * and Barge)
     */
    private void configureAutoBindings() {
        // Auto intake algae intake from reef
        driverController.povUp().whileTrue(buildAutoRoutine(TargetState.ALGAE_BUTTON));

        // Auto barge score
        operatorController.a().whileTrue(buildBargeScoringRoutine());

        // PID + elevator to nearest coral pose left
        operatorController.rightBumper().whileTrue(buildAutoRoutine(TargetState.L2_LEFT));
        operatorController.back().whileTrue(buildAutoRoutine(TargetState.L3_LEFT));
        operatorController.start().whileTrue(buildAutoRoutine(TargetState.L4_LEFT));

        // PID to nearest coral pose left
        operatorController.b().whileTrue(buildAutoRoutine(TargetState.L2_RIGHT));
        operatorController.x().whileTrue(buildAutoRoutine(TargetState.L3_RIGHT));
        operatorController.y().whileTrue(buildAutoRoutine(TargetState.L4_RIGHT));

        // PID to coral source
        operatorController.button(11).whileTrue(buildAutoRoutine(TargetState.LEFT_SOURCE));
        operatorController.button(12).whileTrue(buildAutoRoutine(TargetState.RIGHT_SOURCE));
    }

    /**
     * Binds preset elevator positions to driver controller face buttons and POV.
     */
    private void configureElevatorBindings() {
        // Redundent elevator positions of drive controller
        driverController.a()
                .onTrue(elevatorSubsystem.moveToTargetPosition(Setpoint.kFeederStation).withName("kFeederStation"));
        driverController.b().onTrue(elevatorSubsystem.moveToTargetPosition(Setpoint.kLevel2).withName("kLevel2"));
        driverController.x().onTrue(elevatorSubsystem.moveToTargetPosition(Setpoint.kLevel3).withName("kLevel3"));
        driverController.y().onTrue(elevatorSubsystem.moveToTargetPosition(Setpoint.kLevel4).withName("kLevel4"));

        // Elevator to feeder station after scoring on reef
        operatorController.leftBumper()
                .onTrue(elevatorSubsystem.moveToTargetPosition(Setpoint.kFeederStation).withName("kFeederStation"));

        // Manual elevator movement
        operatorController.axisGreaterThan(1, 0.1).whileTrue(elevatorSubsystem.runMotors(true)
                .withName("runMotorsReverseTrue"));
        operatorController.axisLessThan(1, -0.1).whileTrue(elevatorSubsystem.runMotors(false)
                .withName("runMotorsReverseFalse"));
    }

    /**
     * Configures coral intake and shooting controls.
     */
    private void configureCoralBindings() {
        // Main coral controls
        operatorController.rightStick().whileTrue(coralSubsystem.intakeCoral().withName("intakeCoral"));
        operatorController.leftStick()
                .whileTrue(coralSubsystem.shootCoral().withName("shootCoral").alongWith(buildCoralAssistCommand()));
        thirdController.y().whileTrue(coralSubsystem.shootL1Coral());

        // Redudent coral controls
        driverController.povLeft().whileTrue(coralSubsystem.intakeCoral().withName("intakeCoral"));
        driverController.povRight().whileTrue(coralSubsystem.shootCoral().withName("shootCoral"));
    }

    /**
     * Configures algae intake and shooting controls.
     */
    private void configureAlgaeBindings() {
        // Algae controls
        driverController.leftTrigger().whileTrue(buildAlgaeIntakeRoutine());
        driverController.leftBumper().whileTrue(algaeSubsystem.moveToZero().withName("returnToHomePosAlgae"));
        driverController.rightTrigger().whileTrue(algaeSubsystem.shootAlgae().withName("shootAlgae"));
        driverController.rightTrigger().onFalse(algaeSubsystem.hold(0));
        driverController.povDown().whileTrue(algaeSubsystem.moveToAlgaeShoot());

        // After releasing shoot button, hinge goes to zero position
        operatorController.leftStick().whileFalse(algaeSubsystem.moveToZero());

        // Redudent algae controls
        thirdController.povUp().whileTrue(algaeSubsystem.manualControlForward());
        thirdController.povUp().onFalse(algaeSubsystem.mainMotorHoldCommand());
        thirdController.povDown().whileTrue(algaeSubsystem.manualControlBackwards());
        thirdController.povUp().onFalse(algaeSubsystem.mainMotorHoldCommand());

        thirdController.a().whileTrue(algaeSubsystem.moveToAlgaeShoot());
        thirdController.b().whileTrue(algaeSubsystem.manualIntake());
    }

    /**
     * Sets up the default drive command and fine-adjustment controls.
     */
    private void configureDriveBindings() {
        // Default driving command
        swerveSubsystem.setDefaultCommand(
                new DriveController(swerveSubsystem, () -> {
                    return driverController.getRightX();
                }, () -> {
                    return driverController.getLeftY();
                }, () -> {
                    return driverController.getLeftX();
                },
                        Constants.SwerveModule.Speed.MAX_SPEED));

        // Gyro Reset
        driverController.rightBumper().onTrue(swerveSubsystem.resetGyroCommand());

        // Fine adjustments for coral scoring
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
    }

    /**
     * Registers commands with NamedCommands for pathplanner use.
     */
    private void registerNamedCommands() {
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
    }

    /**
     * Configures the autonomous SendableChooser, filtering by competition options.
     *
     * @return The configured SendableChooser for autonomous command selection.
     */
    private SendableChooser<Command> configureAutoChooser() {
        // For convenience a programmer could change this when going to competition.
        final boolean filterCompetitionAutos = true;

        // Build an auto chooser. This will use Commands.none() as the default option.
        // This will only show autos to select from that start with "Kick" and "Center"
        SendableChooser<Command> chooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
                (stream) -> filterCompetitionAutos
                        ? stream.filter(
                                auto -> (auto.getName().startsWith("Right") || auto.getName().startsWith("Left")
                                        || auto.getName().startsWith("Choreo")))
                        : stream);
        return chooser;
    }

    /**
     * @return The selected autonomous routine in Elastic for execution.
     */
    protected Command getAutonomousCommand() {
        return autoChooser.getSelected();
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
     *
     * <p>
     * Requirements: {@code DriveSubsystem}, {@code ElevatorSubsystem}, and {@code AlgaeSubystem}.
     *
     * @param target The TargetState to reach
     * @return A deferred, composable Command that handles pose filtering, parallel
     *         motion and branch-specific actions.
     */
    public Command buildAutoRoutine(TargetState target) {
        // Defer the entire routine so filterClosestState() is only called once
        return defer(() -> {
            GeneralPose generalPose = swerveSubsystem.filterClosestState(target);

            if (generalPose == null) {
                return new InstantCommand();
            }

            // Base parallel: drive + elevator in parallel
            Command base = parallel(
                    swerveSubsystem.driveToGeneralPose(generalPose),
                    elevatorSubsystem.moveToTargetPosition(generalPose.getTargetState().getSetpoint()));

            // If algae, then add commands onto base
            if (generalPose.getNavTarget() == NavigationTarget.ALGAE) {
                base = base.alongWith(algaeSubsystem.intake())
                        .andThen(swerveSubsystem.backAwayFromReef(generalPose)).alongWith(buildAlgaeIntakeRoutine());
            }

            return base;
        },
                // Declare requirements
                Set.of(swerveSubsystem, elevatorSubsystem, algaeSubsystem));
    }

    /**
     * Builds the routine to score in the barge.
     * Simplified from actual: drive, elevate, and shoot algae.
     */
    private Command buildBargeScoringRoutine() {
        return Commands.sequence(
                // 1) Drive close to barge in parallel with elevator and algae pre-move
                Commands.parallel(
                        swerveSubsystem.driveCloseToBarge()
                                .andThen(swerveSubsystem.stopDriving())
                                .andThen(swerveSubsystem.updateRotationPIDSetpointCommand()),
                        algaeSubsystem.moveToAlgaeShoot(),
                        elevatorSubsystem.moveToTargetPosition(Setpoint.kLevel2)),
                // 2) Ramp elevator to L4, pause, then actually drive to barge
                elevatorSubsystem.moveToTargetPosition(Setpoint.kLevel4),
                new WaitCommand(Constants.Algae.BARGE_L4_WAIT.getValue()),
                swerveSubsystem.driveToBarge()
                        .andThen(swerveSubsystem.stopDriving())
                        .andThen(swerveSubsystem.updateRotationPIDSetpointCommand()),
                new WaitCommand(Constants.Algae.BARGE_SHOOT_WAIT.getValue()),
                // 3) Finally shoot algae
                algaeSubsystem.shootAlgae());
    }

    /**
     * Builds the algae intake routine: intake, wait, move to barge scoring pose.
     */
    private Command buildAlgaeIntakeRoutine() {
        return Commands.sequence(
                algaeSubsystem.intake(),
                new WaitCommand(1.0),
                algaeSubsystem.moveToAlgaeShoot());
    }

    /**
     * Builds a conditional coral assist command when elevator is at L4.
     */
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

    /** Updates the robot's odometry each loop. */
    protected void updateOdometry() {
        this.swerveSubsystem.updateOdometry();
    }

    /**
     * Updates the Real Sense Camera pipeline's robot centric reef pose estimation.
     */
    protected void updateCamera() {
        realSenseCamera.updateReefPose();
    }

    /** Publishes the remaining match time for Elastic. */
    protected void updateMatchTime() {
        SmartDashboard.putNumber("Elastic/Match Time", DriverStation.getMatchTime());
    }

    // protected void updateCamData() {
    // this.cam.updateReefPose();
    // }

    /**
     * (NOT USED CURRENTLY) Zeroes the robot's yaw angle on the field.
     */
    protected void zeroRotation() {
        this.swerveSubsystem.resetGyro();
    }

    /** Refreshes the rotation PID setpoint after auto driving based on PID. */
    protected void updateRotationPIDSetpoint() {
        this.swerveSubsystem.updateRotationPIDSetpoint();
    }

}
