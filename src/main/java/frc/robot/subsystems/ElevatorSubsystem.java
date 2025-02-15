package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkFlex elevatorSecondaryMotor;
    private SparkFlex elevatorPrimaryMotor;
    private SparkClosedLoopController elevatorClosedLoopController;
    private RelativeEncoder elevatorEncoder;
    public static final SparkFlexConfig elevatorPrimaryMotorConfig = new SparkFlexConfig();
    public static final SparkFlexConfig elevatorSecondaryMotorConfig = new SparkFlexConfig();
    private boolean wasResetByButton = false;
    private boolean wasResetByLimit = false;
    private double elevatorCurrentTarget = Setpoint.kFeederStation.encoderPosition;
    private boolean manual = false;

    /** Elevator setpoints */
    public enum Setpoint {
        kFeederStation(Elevator.Positions.FEED_STATION_COUNTS.getValue()),
        kLevel1(Elevator.Positions.L1_COUNTS.getValue()),
        kLevel2(Elevator.Positions.L2_COUNTS.getValue()),
        kLevel3(Elevator.Positions.L3_COUNTS.getValue()),
        kLevel4(Elevator.Positions.L4_COUNTS.getValue());

        public final int encoderPosition;

        Setpoint(int encoderPosition) {
            this.encoderPosition = encoderPosition;
        }
    }

    // Simulation
    private DCMotor elevatorMotorModel = DCMotor.getNeoVortex(1);
    private SparkFlexSim elevatorMotorSim;
    private SparkLimitSwitchSim elevatorLimitSwitchSim;
    public static final double kPixelsPerMeter = 20;
    public static final double kElevatorGearing = 20; // 20:1
    public static final double kCarriageMass = 4.3 + 0.151; // Kg, elevator stage + chain
    public static final double kElevatorDrumRadius = 0.0328 / 2.0; // m
    public static final double kMinElevatorHeightMeters = 0.922; // m
    public static final double kMaxElevatorHeightMeters = 1.62; // m
    private final ElevatorSim elevatorSim = new ElevatorSim(
            elevatorMotorModel,
            kElevatorGearing,
            kCarriageMass,
            kElevatorDrumRadius,
            kMinElevatorHeightMeters,
            kMaxElevatorHeightMeters,
            true,
            kMinElevatorHeightMeters,
            0.0,
            0.0);

    // Mechanism2d setup
    private final Mechanism2d mech2d = new Mechanism2d(50, 50);
    private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 25, 0);
    private final MechanismLigament2d elevatorMech2d = mech2dRoot.append(
            new MechanismLigament2d(
                    "Elevator",
                    kMinElevatorHeightMeters
                            * kPixelsPerMeter,
                    90));

    public ElevatorSubsystem() {
        // elevatorSpeed = Preferences.getDouble("Elevator Motor Speed (rpm)", 1);

        // Elevator Motor
        elevatorPrimaryMotorConfig.idleMode(IdleMode.kBrake)
                //.smartCurrentLimit(Elevator.PrimaryMotor.MAX_CURRENT_IN_A.getValue());
                .voltageCompensation(Elevator.PrimaryMotor.VOLTAGE_COMPENSATION_IN_V.getValue());
        elevatorPrimaryMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control
                .p(Elevator.PrimaryMotor.KP.getValue())
                .d(Elevator.PrimaryMotor.KD.getValue())
                .outputRange(Elevator.PrimaryMotor.MIN_POWER.getValue(),
                        Elevator.PrimaryMotor.MAX_POWER.getValue()).maxMotion
                // Set MAXMotion parameters for position control
                .maxVelocity(Elevator.PrimaryMotor.MAX_VELOCITY_RPM.getValue())
                .maxAcceleration(Elevator.PrimaryMotor.MAX_ACCEL_RPM_PER_S.getValue())
                .allowedClosedLoopError(Elevator.PrimaryMotor.MAX_CONTROL_ERROR_IN_COUNTS.getValue());
        elevatorPrimaryMotorConfig.inverted(Elevator.PrimaryMotor.INVERTED.isTrue());
        System.out.println("Is inverted" + Elevator.PrimaryMotor.INVERTED.isTrue());

        elevatorPrimaryMotor = new SparkFlex(Elevator.PrimaryMotor.CAN_ID.getValue(), MotorType.kBrushless);
        elevatorPrimaryMotor.configure(
                elevatorPrimaryMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        if (Elevator.FOLLOW_DUALENABLE.isTrue()) {
            elevatorSecondaryMotorConfig.idleMode(IdleMode.kCoast);
                    // .smartCurrentLimit(Elevator.PrimaryMotor.MAX_CURRENT_IN_A.getValue())
            //         .voltageCompensation(Elevator.PrimaryMotor.VOLTAGE_COMPENSATION_IN_V.getValue());
            // elevatorSecondaryMotorConfig.closedLoop
            //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //         // Set PID values for position control
            //         .p(Elevator.PrimaryMotor.KP.getValue())
            //         .outputRange(Elevator.PrimaryMotor.MIN_POWER.getValue(),
            //                 Elevator.PrimaryMotor.MAX_POWER.getValue()).maxMotion
            //         // Set MAXMotion parameters for position control
            //         .maxVelocity(Elevator.PrimaryMotor.MAX_VELOCITY_RPM.getValue())
            //         .maxAcceleration(Elevator.PrimaryMotor.MAX_ACCEL_RPM_PER_S.getValue())
            //         .allowedClosedLoopError(Elevator.PrimaryMotor.MAX_CONTROL_ERROR_IN_COUNTS.getValue());
            elevatorSecondaryMotorConfig.follow(Elevator.PrimaryMotor.CAN_ID.getValue(), true);

            elevatorSecondaryMotor = new SparkFlex(Elevator.SecondaryMotor.CAN_ID.getValue(), MotorType.kBrushless);
            elevatorSecondaryMotor.configure(
                    elevatorSecondaryMotorConfig,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
        }

        elevatorClosedLoopController = elevatorPrimaryMotor.getClosedLoopController();
        elevatorEncoder = elevatorPrimaryMotor.getEncoder();

        // Simulation stuff
        elevatorLimitSwitchSim = new SparkLimitSwitchSim(elevatorPrimaryMotor, false);
        elevatorMotorSim = new SparkFlexSim(elevatorPrimaryMotor, elevatorMotorModel);
    }

    public void periodic() {
        if(!manual){
            moveToSetpoint();
        }
        //zeroElevatorOnLimitSwitch();
        zeroOnUserButton();

        // Update mechanism2d
        elevatorMech2d.setLength(
                kPixelsPerMeter * kMinElevatorHeightMeters
                        + kPixelsPerMeter
                                * (elevatorEncoder.getPosition() / kElevatorGearing)
                                * (kElevatorDrumRadius * 2.0 * Math.PI));

        SmartDashboard.putString("Elevator/Target Position", String.valueOf(elevatorCurrentTarget));
        SmartDashboard.putNumber("Elevator/Target Position Encoder", elevatorCurrentTarget);
        SmartDashboard.putNumber("Elevator/Actual Position", elevatorEncoder.getPosition());
        SmartDashboard.putData("Elevator/Model", mech2d);
    }

    /**
     * Command to set the subsystem setpoint. This will set the arm and elevator to
     * their predefined
     * positions for the given setpoint.
     */
    public Command moveToTargetPosition(Setpoint setpoint) {
        return this.runOnce(
                () -> {
                    switch (setpoint) {
                        case kFeederStation:
                            elevatorCurrentTarget = Setpoint.kFeederStation.encoderPosition;
                            break;
                        case kLevel1:
                            elevatorCurrentTarget = Setpoint.kLevel1.encoderPosition;
                            break;
                        case kLevel2:
                            elevatorCurrentTarget = Setpoint.kLevel2.encoderPosition;
                            break;
                        case kLevel3:
                            elevatorCurrentTarget = Setpoint.kLevel3.encoderPosition;
                            break;
                        case kLevel4:
                            elevatorCurrentTarget = Setpoint.kLevel4.encoderPosition;
                            break;
                    }
                });
    }

    /**
     * Drive the arm and elevator motors to their respective setpoints. This will
     * use MAXMotion
     * position control which will allow for a smooth acceleration and deceleration
     * to the mechanisms'
     * setpoints.
     */
    private void moveToSetpoint() {
        elevatorClosedLoopController.setReference(
                elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
    }

    /** Zero the elevator encoder when the limit switch is pressed. */
    private void zeroElevatorOnLimitSwitch() {
        if (!wasResetByLimit && elevatorPrimaryMotor.getReverseLimitSwitch().isPressed()) {
            // Zero the encoder only when the limit switch is switches from "unpressed" to
            // "pressed" to prevent constant zeroing while pressed
            elevatorEncoder.setPosition(0);
            wasResetByLimit = true;
            // System.out.println(true);
        } else if (!elevatorPrimaryMotor.getReverseLimitSwitch().isPressed()) {
            wasResetByLimit = false;
        }
    }

    /**
     * Zero the arm and elevator encoders when the user button is pressed on the
     * roboRIO.
     */
    private void zeroOnUserButton() {
        if (!wasResetByButton && RobotController.getUserButton()) {
            // Zero the encoders only when button switches from "unpressed" to "pressed" to
            // prevent
            // constant zeroing while pressed
            wasResetByButton = true;
            elevatorEncoder.setPosition(0);
            elevatorCurrentTarget = 0;
        } else if (!RobotController.getUserButton()) {
            wasResetByButton = false;
        }
    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        elevatorSim.setInput(elevatorPrimaryMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

        // Update sim limit switch
        elevatorLimitSwitchSim.setPressed(elevatorSim.getPositionMeters() >= 0 && elevatorSim.getPositionMeters() <= 5);

        // Next, we update it. The standard loop time is 20ms.
        elevatorSim.update(0.020);

        // Iterate the elevator and arm SPARK simulations
        elevatorMotorSim.iterate(
                ((elevatorSim.getVelocityMetersPerSecond()
                        / (kElevatorDrumRadius * 2.0 * Math.PI))
                        * kElevatorGearing)
                        * 60.0,
                RobotController.getBatteryVoltage(),
                0.02);
    }

    public Command runMotors(boolean reverse) {
        double speed = reverse ? Elevator.PrimaryMotor.LOWER_SPEED.getValue() * -1: Elevator.PrimaryMotor.RAISE_SPEED.getValue() * 1;
        return runEnd(
                () -> {
                    elevatorPrimaryMotor.set(speed);
                    manual = true;
                },
                () -> {
                    elevatorPrimaryMotor.stopMotor();
                    manual = false;
                    elevatorCurrentTarget = elevatorEncoder.getPosition();
                });
    }

}
