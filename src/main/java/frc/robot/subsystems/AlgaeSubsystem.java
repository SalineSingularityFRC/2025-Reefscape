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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AlgaeSubsystem extends SubsystemBase {
    private TalonFX armRotatorMotor, rollerMotor;
    private LaserCan sensor;
    private double sensingDistance;
    private final VelocityTorqueCurrentFOC intakeSpeedRequest, outtakeSpeedRequest, zeroSpeedRequest;
    private PositionTorqueCurrentFOC rollerMotorHoldRequest;
    private final PositionTorqueCurrentFOC armRotatorMotorRequest;
    private final PositionTorqueCurrentFOC armRotatorMotorAlgaeRequest;
    private final PositionTorqueCurrentFOC armRotatorMotorZeroRequest;
    private static final int LASER_CAN_NO_MEASUREMENT = -1;
    private CANcoder canCoder;
    private boolean manual = false;
    State state;

    public AlgaeSubsystem() {
        armRotatorMotor = new TalonFX(Constants.CanId.Algae.MAIN_MOTOR);
        rollerMotor = new TalonFX(Constants.CanId.Algae.ALGAE_MOTOR);
        sensor = new LaserCan(Constants.CanId.Algae.ALGAE_LASER);
        canCoder = new CANcoder(Constants.CanId.CanCoder.ALGAE);

        intakeSpeedRequest = new VelocityTorqueCurrentFOC(Constants.Algae.motorSpeedSlow.getValue()).withSlot(0);
        zeroSpeedRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
        outtakeSpeedRequest = new VelocityTorqueCurrentFOC(Constants.Algae.motorSpeedFast.getValue()).withSlot(0);
        rollerMotorHoldRequest = new PositionTorqueCurrentFOC(rollerMotor.getPosition().getValueAsDouble()).withSlot(1);
        armRotatorMotorRequest = new PositionTorqueCurrentFOC(armRotatorMotor.getPosition().getValueAsDouble()).withSlot(0);
        armRotatorMotorAlgaeRequest = new PositionTorqueCurrentFOC(armRotatorMotor.getPosition().getValueAsDouble()).withSlot(1);
        armRotatorMotorZeroRequest = new PositionTorqueCurrentFOC(armRotatorMotor.getPosition().getValueAsDouble()).withSlot(2);

        canCoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(
                new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.Clockwise_Positive)));

        armRotatorMotor.setPosition(0);

        armRotatorMotor.setNeutralMode(NeutralModeValue.Brake);
        armRotatorMotor.getConfigurator().apply(new TalonFXConfiguration());

        rollerMotor.setNeutralMode(NeutralModeValue.Brake);
        rollerMotor.getConfigurator().apply(new TalonFXConfiguration());

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
        armRotatorMotor.getConfigurator().apply(mainConfig);

        // For algae intake motor
        TalonFXConfiguration algaeConfig = new TalonFXConfiguration();
        algaeConfig.Slot0.kP = Constants.Algae.kPAlgae.getValue();
        algaeConfig.Slot0.kD = Constants.Algae.kDAlgae.getValue();
        algaeConfig.Slot1.kP = Constants.Algae.kP1Algae.getValue();
        algaeConfig.Slot1.kD = Constants.Algae.kD1Algae.getValue();
        rollerMotor.getConfigurator().apply(algaeConfig);

        sensingDistance = Constants.Algae.sensingDistance.getValue();

        setDefaultCommand(holdCommand());

        this.state = State.NEUTRAL;
    }

    public void periodic() {
        // if (canSeeAlgae() && timer.hasElapsed(2)) {
        // hasAlgae = true;
        // }
        //
        boolean status = true;
        if (!manual) {
            // mainMotor.setControl(mainMotorHoldRequest);
            // status = algaeMotor.setControl(algaeMotorHoldRequest).isOK();
        }

        // intakeSpeedRequest.Velocity = Constants.Algae.motorSpeedSlow.getValue();
        // outtakeSpeedRequest.Velocity = Constants.Algae.motorSpeedFast.getValue();

        SmartDashboard.putNumber("AlgaeRotater/Position", rollerMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("AlgaeRotater/HoldPosition", rollerMotorHoldRequest.Position);
        SmartDashboard.putBoolean("AlgaeRotater/Manual", manual);
        SmartDashboard.putBoolean("AlgaeRotater/Status", status);

        SmartDashboard.putNumber("AlgaeHinge/TargetPosition", armRotatorMotorRequest.Position);
        SmartDashboard.putNumber("AlgaeHinge/TargetPositionForZero", armRotatorMotorZeroRequest.Position);
        SmartDashboard.putNumber("AlgaeHinge/ActualPosition", armRotatorMotor.getPosition().getValueAsDouble());

        switch (state){
            case SHOOT_ALGAE:
                armRotatorMotorRequest.Position = Constants.Algae.SHOOT_POS.getValue();
                armRotatorMotor.setControl(armRotatorMotorRequest);
                rollerMotor.setControl(outtakeSpeedRequest);
                break;
            case SHOOT_POSE:
                //Different PIDs for if have algae or not
                if (canSeeAlgae()) {
                    armRotatorMotorAlgaeRequest.Position = Constants.Algae.SHOOT_POS.getValue();
                    armRotatorMotor.setControl(armRotatorMotorAlgaeRequest);

                } else {
                    armRotatorMotorRequest.Position = Constants.Algae.SHOOT_POS.getValue();
                    armRotatorMotor.setControl(armRotatorMotorRequest);
                }

                rollerMotor.setControl(rollerMotorHoldRequest);
                break;
            case INTAKE_ALGAE:
                armRotatorMotorRequest.Position = Constants.Algae.INTAKE_POS.getValue();
                armRotatorMotor.setControl(armRotatorMotorRequest);
                rollerMotor.setControl(intakeSpeedRequest);
                break;
            case NEUTRAL:
                armRotatorMotorZeroRequest.Position = Constants.Algae.DEFAULT_POSE.getValue();
                armRotatorMotor.setControl(armRotatorMotorZeroRequest);
                rollerMotorHoldRequest.Position = rollerMotor.getPosition().getValueAsDouble();
                rollerMotor.setControl(rollerMotorHoldRequest);
                break;
            case CORAL_KICK:
                armRotatorMotorRequest.Position = Constants.Algae.CORAL_SCORE_POSE.getValue();
                armRotatorMotor.setControl(armRotatorMotorRequest);
        }
    }

    public enum State{
        SHOOT_ALGAE, SHOOT_POSE, INTAKE_ALGAE, NEUTRAL, CORAL_KICK
    }

    public Command holdCommand() {
        return run(() -> {
        });
    }

    public Command shoot() {
        return run(() -> {
            this.state = State.SHOOT_ALGAE;
        })

        .until(() -> !canSeeAlgae())
        .andThen(new WaitCommand(0.05))
        .andThen( runOnce(() -> {
            this.state = State.NEUTRAL;
        }));
    }

    public Command hold() {
        return runOnce(() -> {
            rollerMotorHoldRequest.Position = 0;
            rollerMotor.setPosition(0);
        });
    }

    public Command intake() {
        return run(() -> {
            this.state = State.INTAKE_ALGAE;
        })
            .until(() -> canSeeAlgae())
            .andThen(afterIntakeButtonSequence());
    }

    public Command afterIntakeButtonSequence() {
        return new SequentialCommandGroup(
            new WaitCommand(0.05),
            hold(0.5),
            new WaitCommand(1.0),
            armConditionalMove()
        );
    }

    /**
     * Builds the algae intake routine: wait, move to barge scoring pose.
     */
    public Command afterIntakeAutoSequence() {
        return new SequentialCommandGroup(
            new WaitCommand(1.0),
            armConditionalMove()
        );
    }

    // Zero pose for arm motor
    public Command setNeutralState(){
        return runOnce(() -> {
            this.state = State.NEUTRAL;
        });
    }

    // Kick for coral scoring
    public Command setCoralKickState(){
        return runOnce(() -> {
            this.state = State.CORAL_KICK;
        });
    }

    // Shoot algae
    public Command setShootAlgaeState(){
        return runOnce(() -> {
            this.state = State.SHOOT_ALGAE;
        });
    }


    // Arm motor goes to shoot pose
    public Command setShootPoseState(){
        return runOnce(() -> {
            this.state = State.SHOOT_POSE;
        });
    }

    public Command armConditionalMove() {
        return runOnce(() -> {
            if (canSeeAlgae()) {
                this.state = State.SHOOT_POSE;
            } else {
                this.state = State.NEUTRAL;
            }
        });
    }

    public Command hold(double extra) {
        return runOnce(() -> {
            rollerMotorHoldRequest.Position = extra;
            rollerMotor.setPosition(0);
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
}