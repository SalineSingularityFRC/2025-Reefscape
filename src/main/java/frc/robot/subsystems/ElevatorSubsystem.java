package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax elevatorMotor;
    private SparkClosedLoopController elevatorClosedLoopController;
    private RelativeEncoder elevatorEncoder;
    public static final SparkMaxConfig liftMotorConfig = new SparkMaxConfig();
    private boolean wasResetByButton = false;
    private boolean wasResetByLimit = false;
    private Setpoint elevatorCurrentTarget = Setpoint.kFeederStation;

    /** Elevator setpoints */
    public enum Setpoint {
        kFeederStation(0),
        kLevel1(100),
        kLevel2(200),
        kLevel3(300),
        kLevel4(400);

        public final int encoderPosition;

        Setpoint(int encoderPosition) {
            this.encoderPosition = encoderPosition;
        }
    }

    // Simulation
    private DCMotor elevatorMotorModel = DCMotor.getNeoVortex(1);
    private SparkMaxSim elevatorMotorSim;
    private SparkLimitSwitchSim elevatorLimitSwitchSim;
    public static final double kPixelsPerMeter = 20;
    public static final double kElevatorGearing = 25; // 25:1
    public static final double kCarriageMass = 4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
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

        // Left Motor
        double intakeLeftConfigP = Preferences.getDouble("Elevator kP", .1);
        double intakeLeftConfigPowerMin = Preferences.getDouble("Elevator Min Power", -1);
        double intakeLeftConfigPowerMax = Preferences.getDouble("Elevator Max Power", 1);
        double intakeLeftConfigMaxV = Preferences.getDouble("Elevator Max Velocity (rpm-ish)", 2000);
        double intakeLeftConfigMaxA = Preferences.getDouble("Elevator Max Accel (rpm/s-ish)", 10000);
        int intakeLeftConfigSmartCurrentLimitAmps = Preferences.getInt("Elevator Max Current (A)", 40);
        double intakeLeftConfigVoltageCompensation = Preferences.getDouble("Elevator Voltage Compensation (V)", 12);
        double intakeLeftConfigMaxClosedLoopError = Preferences.getDouble("Elevator Max Error", 0.25);
        liftMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(intakeLeftConfigSmartCurrentLimitAmps)
                .voltageCompensation(intakeLeftConfigVoltageCompensation);
        liftMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control
                .p(intakeLeftConfigP)
                .outputRange(intakeLeftConfigPowerMin, intakeLeftConfigPowerMax).maxMotion
                // Set MAXMotion parameters for position control
                .maxVelocity(intakeLeftConfigMaxV)
                .maxAcceleration(intakeLeftConfigMaxA)
                .allowedClosedLoopError(intakeLeftConfigMaxClosedLoopError);
        elevatorMotor = new SparkMax(Constants.CanId.Elevator.ELEVATOR_MOTOR, MotorType.kBrushless);
        elevatorMotor.configure(
                liftMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        elevatorClosedLoopController = elevatorMotor.getClosedLoopController();
        elevatorEncoder = elevatorMotor.getEncoder();

        // Simulation stuff
        elevatorLimitSwitchSim = new SparkLimitSwitchSim(elevatorMotor, false);
        elevatorMotorSim = new SparkMaxSim(elevatorMotor, elevatorMotorModel);
    }

    public void periodic() {
        moveToSetpoint();
        zeroElevatorOnLimitSwitch();
        zeroOnUserButton();

        // Update mechanism2d
        elevatorMech2d.setLength(
                kPixelsPerMeter * kMinElevatorHeightMeters
                        + kPixelsPerMeter
                                * (elevatorEncoder.getPosition() / kElevatorGearing)
                                * (kElevatorDrumRadius * 2.0 * Math.PI));

        SmartDashboard.putString("Elevator/Target Position", String.valueOf(elevatorCurrentTarget));
        SmartDashboard.putNumber("Elevator/Target Position Encoder", elevatorCurrentTarget.encoderPosition);
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
                            elevatorCurrentTarget = Setpoint.kFeederStation;
                            break;
                        case kLevel1:
                            elevatorCurrentTarget = Setpoint.kLevel1;
                            break;
                        case kLevel2:
                            elevatorCurrentTarget = Setpoint.kLevel2;
                            break;
                        case kLevel3:
                            elevatorCurrentTarget = Setpoint.kLevel3;
                            break;
                        case kLevel4:
                            elevatorCurrentTarget = Setpoint.kLevel4;
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
                elevatorCurrentTarget.encoderPosition, ControlType.kMAXMotionPositionControl);
    }

    /** Zero the elevator encoder when the limit switch is pressed. */
    private void zeroElevatorOnLimitSwitch() {
        if (!wasResetByLimit && elevatorMotor.getReverseLimitSwitch().isPressed()) {
            // Zero the encoder only when the limit switch is switches from "unpressed" to
            // "pressed" to prevent constant zeroing while pressed
            elevatorEncoder.setPosition(0);
            wasResetByLimit = true;
        } else if (!elevatorMotor.getReverseLimitSwitch().isPressed()) {
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
        } else if (!RobotController.getUserButton()) {
            wasResetByButton = false;
        }
    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        elevatorSim.setInput(elevatorMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

        // Update sim limit switch
        elevatorLimitSwitchSim.setPressed(elevatorSim.getPositionMeters() == 0);

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

}
