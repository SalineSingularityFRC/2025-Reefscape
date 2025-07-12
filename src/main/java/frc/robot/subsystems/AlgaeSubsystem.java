package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AlgaeSubsystem extends SubsystemBase {
    private TalonFX mainMotor, algaeMotor;
    private LaserCan sensor;
    private double sensingDistance;
    private final VelocityTorqueCurrentFOC intakeSpeedRequest, outtakeSpeedRequest, zeroSpeedRequest;
    private PositionTorqueCurrentFOC algaeMotorHoldRequest;
    private final PositionTorqueCurrentFOC mainMotorRequest;
    private final PositionTorqueCurrentFOC mainMotorAlgaeRequest;
    private final PositionTorqueCurrentFOC mainMotorZeroRequest;
    private static final int LASER_CAN_NO_MEASUREMENT = -1;
    private boolean hasAlgae;
    private double targetPosition;
    private CANcoder canCoder;
    private boolean manual = false;

    public AlgaeSubsystem() {
        mainMotor = new TalonFX(Constants.CanId.Algae.MAIN_MOTOR);
        algaeMotor = new TalonFX(Constants.CanId.Algae.ALGAE_MOTOR);
        sensor = new LaserCan(Constants.CanId.Algae.ALGAE_LASER);
        canCoder = new CANcoder(Constants.CanId.CanCoder.ALGAE);

        intakeSpeedRequest = new VelocityTorqueCurrentFOC(Constants.Algae.motorSpeedSlow.getValue()).withSlot(0);
        zeroSpeedRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
        outtakeSpeedRequest = new VelocityTorqueCurrentFOC(Constants.Algae.motorSpeedFast.getValue()).withSlot(0);
        algaeMotorHoldRequest = new PositionTorqueCurrentFOC(algaeMotor.getPosition().getValueAsDouble()).withSlot(1);
        mainMotorRequest = new PositionTorqueCurrentFOC(mainMotor.getPosition().getValueAsDouble()).withSlot(0);
        mainMotorAlgaeRequest = new PositionTorqueCurrentFOC(mainMotor.getPosition().getValueAsDouble()).withSlot(1);
        mainMotorZeroRequest = new PositionTorqueCurrentFOC(mainMotor.getPosition().getValueAsDouble()).withSlot(2);

        canCoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(
                new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.Clockwise_Positive)));

        mainMotor.setPosition(0);

        mainMotor.setNeutralMode(NeutralModeValue.Brake);
        mainMotor.getConfigurator().apply(new TalonFXConfiguration());

        algaeMotor.setNeutralMode(NeutralModeValue.Brake);
        algaeMotor.getConfigurator().apply(new TalonFXConfiguration());

        // No algae inside PIDs
        TalonFXConfiguration mainConfig = new TalonFXConfiguration();
        mainConfig.Slot0.kP = Constants.Algae.kPMain.getValue();
        mainConfig.Slot0.kD = Constants.Algae.kDMain.getValue();
        mainConfig.Slot0.kS = Constants.Algae.kSMain.getValue();
        mainConfig.Slot0.kG = Constants.Algae.kVMain.getValue();

        // Algae Inside PIDs
        mainConfig.Slot1.kP = Constants.Algae.kPMainAlgaeInside.getValue();
        mainConfig.Slot1.kI = Constants.Algae.kIMainAlgaeInside.getValue();
        mainConfig.Slot1.kD = Constants.Algae.kDMainAlgaeInside.getValue();
        mainConfig.Slot1.kS = Constants.Algae.kSMainAlgaeInside.getValue();
        mainConfig.Slot1.kG = Constants.Algae.kVMainAlgaeInside.getValue();

        // Algae Zero PIDs
        mainConfig.Slot2.kP = Constants.Algae.kPMainAlgaeDown.getValue();
        mainConfig.Slot2.kD = Constants.Algae.kDMainAlgaeDown.getValue();
        mainConfig.Slot2.kS = Constants.Algae.kSMainAlgaeDown.getValue();
        mainConfig.Slot2.kG = Constants.Algae.kVMainAlgaeDown.getValue();

        mainConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        mainConfig.CurrentLimits.SupplyCurrentLimit = 5;
        mainConfig.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
        mainConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        mainMotor.getConfigurator().apply(mainConfig);

        // For algae intake motor
        TalonFXConfiguration algaeConfig = new TalonFXConfiguration();
        algaeConfig.Slot0.kP = Constants.Algae.kPAlgae.getValue();
        algaeConfig.Slot0.kD = Constants.Algae.kDAlgae.getValue();
        algaeConfig.Slot1.kP = Constants.Algae.kP1Algae.getValue();
        algaeConfig.Slot1.kD = Constants.Algae.kD1Algae.getValue();
        algaeMotor.getConfigurator().apply(algaeConfig);

        sensingDistance = Constants.Algae.sensingDistance.getValue();

        hasAlgae = false;

        setDefaultCommand(holdCommand());
    }

    public void periodic() {
        // if (canSeeAlgae() && timer.hasElapsed(2)) {
        // hasAlgae = true;
        // }
        //
        boolean status = true;
        if (!manual) {
            // mainMotor.setControl(mainMotorHoldRequest);
            status = algaeMotor.setControl(algaeMotorHoldRequest).isOK();
        }

        intakeSpeedRequest.Velocity = Constants.Algae.motorSpeedSlow.getValue();
        outtakeSpeedRequest.Velocity = Constants.Algae.motorSpeedFast.getValue();

        SmartDashboard.putNumber("AlgaeRotater/Position", algaeMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("AlgaeRotater/HoldPosition", algaeMotorHoldRequest.Position);
        SmartDashboard.putBoolean("AlgaeRotater/Manual", manual);
        SmartDashboard.putBoolean("AlgaeRotater/Status", status);

        SmartDashboard.putNumber("AlgaeHinge/TargetPosition", mainMotorRequest.Position);
        SmartDashboard.putNumber("AlgaeHinge/TargetPositionForZero", mainMotorZeroRequest.Position);
        SmartDashboard.putNumber("AlgaeHinge/ActualPosition", mainMotor.getPosition().getValueAsDouble());
    }

    public Command holdCommand() {
        return run(() -> {
        });
    }

    public Command moveToPos(DoubleSupplier targetPos) {
        return new FunctionalCommand(
                () -> {
                    mainMotorRequest.Position = targetPos.getAsDouble();
                    mainMotorAlgaeRequest.Position = targetPos.getAsDouble();
                },
                () -> {
                    if (canSeeAlgae()) {
                        mainMotor.setControl(mainMotorAlgaeRequest);
                    } else {
                        mainMotor.setControl(mainMotorRequest);
                    }
                },
                (_unused) -> {

                },
                () -> {
                    return Math.abs(targetPos.getAsDouble()
                            - mainMotor.getPosition().getValueAsDouble()) < Constants.Algae.MAX_CONTROL_ERROR_IN_COUNTS
                                    .getValue();
                });
    }

    // Uses diff PIDs than moveToPos
    public Command moveToPosZero(DoubleSupplier targetPos) {
        return new FunctionalCommand(
                () -> {
                    mainMotorZeroRequest.Position = targetPos.getAsDouble();
                },
                () -> {
                    mainMotor.setControl(mainMotorZeroRequest);
                },
                (_unused) -> {

                },
                () -> {
                    return Math.abs(targetPos.getAsDouble()
                            - mainMotor.getPosition().getValueAsDouble()) < Constants.Algae.MAX_CONTROL_ERROR_IN_COUNTS
                                    .getValue();
                });
    }

    public Command moveToIntakePos() {
        return moveToPos(() -> Constants.Algae.INTAKE_POS.getValue());
    }

    public Command moveToCoralScorePose() {
        return moveToPos(() -> Constants.Algae.CORAL_SCORE_POSE.getValue());
    }

    // Todo: get rid of pose supplier
    public Command moveToZero() {
        return moveToPosZero(() -> Constants.Algae.DEFAULT_POSE.getValue());
    }

    public Command shootAlgae() {
        return runMotorsToSpit().until(() -> !canSeeAlgae())
                .andThen(new WaitCommand(0.05)).andThen(hold(0));
    }

    public Command moveToAlgaeShoot() {
        return moveToPos(() -> Constants.Algae.SHOOT_POS.getValue());
    }

    public Command intake() {
        return moveToIntakePos()
                .alongWith(runMotorsToIntake().until(() -> canSeeAlgae())
                .andThen(new WaitCommand(0.05))
                .andThen(hold(0.5)));
    }

    public Command manualIntake() {
        return moveToIntakePos()
                .alongWith(runMotorsToIntake())
                .andThen(new WaitCommand(0.05))
                .andThen(hold(0.5));
    }

    public Command runMotorsToIntake() {
        return runEnd(() -> {
            manual = true;
            algaeMotor.setControl(intakeSpeedRequest); // If algae is in system, make motor speed fast, otherwise slow
        }, () -> {
            // mainMotor.setControl(new DutyCycleOut(0));
            // algaeMotor.setControl(zeroSpeedRequest.withSlot(0));
            manual = false;
            algaeMotorHoldRequest.Position = algaeMotor.getPosition().getValueAsDouble();
            // SmartDashboard.putNumber(getName(), LASER_CAN_NO_MEASUREMENT)
            algaeMotor.setControl(algaeMotorHoldRequest.withSlot(1));
        });
    }

    public Command runMotorsToSpit() {
        return runEnd(() -> {
            algaeMotor.setControl(outtakeSpeedRequest);
        }, () -> {
            algaeMotor.setControl(zeroSpeedRequest.withSlot(0));
            manual = false;
        });
    }

    public Command hold(double extra) {
        return runOnce(() -> {
            algaeMotorHoldRequest.Position = algaeMotor.getPosition().getValueAsDouble() + extra;
        });
    }

    // public Command stopAlgaeMotor() {
    // return runOnce(() -> {
    // algaeMotor.stopMotor();
    // });
    // }

    public boolean canSeeAlgae() {
        SmartDashboard.putBoolean("Algae/Can see algae",
                sensor.getMeasurement() != null && getSensorValue(sensor) <= sensingDistance);
        SmartDashboard.putNumber("Algae/Algae Sensor Distnace", getSensorValue(sensor));        
        return sensor.getMeasurement() != null && getSensorValue(sensor) <= sensingDistance;
    }

    public int getSensorValue(LaserCan laserCan) {
        Measurement measurement = laserCan.getMeasurement();
        return measurement == null ? LASER_CAN_NO_MEASUREMENT : measurement.distance_mm;
    }

    public Command returnToHomePos() {
        return moveToPos(() -> 0).onlyIf(() -> !canSeeAlgae());
    }

    public Command mainMotorHoldCommand() {
        return runOnce(() -> {
            mainMotorRequest.Position = mainMotor.getPosition().getValueAsDouble();
            this.manual = false;
        });
    }

    public Command manualControlForward() {
        return runEnd(() -> {
            this.manual = true;
            mainMotor.setControl(new DutyCycleOut(Constants.Algae.manualSpeed.getValue()));
        }, () -> {
            // mainMotor.setControl(new DutyCycleOut(0));
            mainMotorHoldCommand();
            this.manual = false;
        });
    }

    public Command manualControlBackwards() {
        return runEnd(() -> {
            this.manual = true;
            mainMotor.setControl(new DutyCycleOut(-Constants.Algae.manualSpeed.getValue()));
        }, () -> {
            // mainMotor.setControl(new DutyCycleOut(0));
            mainMotorHoldCommand();
        });
    }
}

// class intakeCoral extends ParallelCommandGroup {
//     intakeCoral() {
//         addCommands(null);
//     }
// }