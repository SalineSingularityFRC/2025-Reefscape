package frc.robot.SwerveClasses;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.Constants;
import frc.robot.PID;
import java.lang.Math;

/*
 * This class owns the components of a single swerve module and is responsible for controlling
 * the angle and speed that the module is moving
 */
public class SwerveModule {

    /*
     * We will need a couple different instance variables
     * An instance of the SwerveAngle class to handle the angle motor
     * An instance of the TalonFX class to handle the drive motor
     * An instance of the CANcoder class to handle the encoder
     */
    private SwerveAngle angleMotor;
    private CANcoder c_encoder;
    private TalonFX driveMotor;
    private MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    private VelocityTorqueCurrentFOC velocityTarget;
    private BaseStatusSignal[] m_signals;
    private StatusSignal<Angle> m_drivePosition;
    private StatusSignal<AngularVelocity> m_driveVelocity;
    private StatusSignal<Angle> m_steerPosition;
    private StatusSignal<AngularVelocity> m_steerVelocity;
    private SwerveModulePosition m_internalState = new SwerveModulePosition();
    private double m_driveRotationsPerMeter;

    private final double absolutePositionEncoderOffset;
    private String name;

    /*
     * This constructor needs to take two parameters, one for the CAN ID of the
     * drive motor and one for the CAN ID of the
     * angle motor
     * It should initialize our drive motor and create a SwerveAngle, passing the
     * CAN ID to the SwerveAngle constructor
     */
    public SwerveModule(
            int Can_ID_driveMotor,
            int Can_ID_angleMotor,
            int Can_ID_encoder,
            double zeroPosition,
            String canNetwork,
            boolean isInverted,
            boolean isInvertedAngle,
            String name) { // add a zeroPosition thing
        c_encoder = new CANcoder(Can_ID_encoder, canNetwork);

        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cancoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1);
        c_encoder.getConfigurator().apply(cancoderConfig);

        driveMotor = new TalonFX(Can_ID_driveMotor, canNetwork);

        // Factory reset before applying configs
        driveMotor.getConfigurator().apply(new TalonFXConfiguration());

        CurrentLimitsConfigs current = new CurrentLimitsConfigs();
        current.SupplyCurrentLimit = 25;
        current.SupplyCurrentLimitEnable = true;
        driveMotor.getConfigurator().apply(current);
        angleMotor = new SwerveAngle(Can_ID_angleMotor, canNetwork, isInvertedAngle);

        this.name = name;
        driveMotor.setInverted(isInverted);
        if (isInverted) {
            motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        } else {
            motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        setBrakeMode();

        absolutePositionEncoderOffset = zeroPosition;
        this.resetZeroAngle();

        m_drivePosition = driveMotor.getPosition();
        m_driveVelocity = driveMotor.getVelocity();
        m_steerPosition = c_encoder.getAbsolutePosition();
        m_steerVelocity = c_encoder.getVelocity();

        m_signals = new BaseStatusSignal[4];
        m_signals[0] = m_drivePosition;
        m_signals[1] = m_driveVelocity;
        m_signals[2] = m_steerPosition;
        m_signals[3] = m_steerVelocity;

        /*
         * Calculate the ratio of drive motor rotation to meter on ground
         */
        double rotationsPerWheelRotation = Constants.SwerveModule.GearRatio.DRIVE;
        double metersPerWheelRotation = 2 * Math.PI * Constants.Measurement.WHEELRADIUS;
        m_driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;

        // For drive PIDs
        velocityTarget = new VelocityTorqueCurrentFOC(0).withSlot(0).withFeedForward(0);

        PID drivePID = Constants.PidGains.SwerveModule.DRIVE_PID_CONTROLLER;
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = drivePID.P;
        slot0Configs.kI = drivePID.I;
        slot0Configs.kD = drivePID.D;
        slot0Configs.kS = drivePID.S;

        driveMotor.getConfigurator().apply(slot0Configs);
    }

    public SwerveModulePosition getPosition(boolean refresh) {
        if (refresh) {
            /* Refresh all signals */
            m_drivePosition.refresh();
            m_driveVelocity.refresh();
            m_steerPosition.refresh();
            m_steerVelocity.refresh();
        }

        /* Now latency-compensate our signals */
        Measure<AngleUnit> drive_rot = BaseStatusSignal.getLatencyCompensatedValue(m_drivePosition, m_driveVelocity);
        Measure<AngleUnit> angle_rot = BaseStatusSignal.getLatencyCompensatedValue(m_steerPosition, m_steerVelocity);

        /*
         * Convert signals into rotations so we can use them
         */
        double drive_rot_rotations = drive_rot.in(edu.wpi.first.units.Units.Rotations);
        double angle_rot_rotations = angle_rot.in(edu.wpi.first.units.Units.Rotations);

        /* And push them into a SwerveModuleState object to return */
        m_internalState.distanceMeters = drive_rot_rotations / m_driveRotationsPerMeter;
        /* Angle is already in terms of steer rotations */
        m_internalState.angle = Rotation2d.fromRotations(angle_rot_rotations - absolutePositionEncoderOffset);

        return m_internalState;
    }

    public SwerveModuleState getState() {
        m_driveVelocity.refresh(); // May not need to?
        return new SwerveModuleState(m_driveVelocity.getValueAsDouble() / m_driveRotationsPerMeter,
                m_internalState.angle);
    }

    protected void coast() {
        // driveMotor.set(0); // this is for when the joystick is not being moved at all
        driveMotor.stopMotor();
    }

    /*
     * Set the zero angle based on the current angle (in radians) that we are
     * reading from an external source(absolute encoder).
     * We should be able to read the current angle from the CANcoder and pass that
     * to the setZeroAngle method in
     * the SwerveAngle class
     * The can is where we base all of our correct angles on.
     * The talon says we are at an angle but sometimes that might not be the right
     * angle.
     * The zeroAngle is what we use to offset(balance) whatever we're reading off
     * the talon
     */
    protected void resetZeroAngle() {
        angleMotor.setZeroAngle(m_internalState.angle.getRadians());
    }

    public double getAngleClamped() {
        return angleMotor.getAngleClamped();
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getEncoderPosition()));
        state.cosineScale(new Rotation2d(getEncoderPosition()));

        SmartDashboard.putNumber("Target Velocity" + name, toRotationsPerSecond(state));
        SmartDashboard.putNumber("Current Velocity" + name, driveMotor.getVelocity().getValueAsDouble());

        switch(angleMotor.setAngle(state.angle.getRadians(), name)){
        case Positive:
            driveMotor.setControl(velocityTarget.withVelocity(toRotationsPerSecond(state)));
            break;
        case Negative:
            driveMotor.setControl(velocityTarget.withVelocity(-toRotationsPerSecond(state)));
            break;
        default:
            break;
        }
    }    

    public void setCoastMode() {
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(motorOutputConfigs);
    }

    public void setBrakeMode() {
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        driveMotor.getConfigurator().apply(motorOutputConfigs);
    }

    public boolean isCoast() {
        if (motorOutputConfigs.NeutralMode == NeutralModeValue.Coast) {
            return true;
        } else {
            return false; // it is brake mode
        }
    }

    // Conversion to get in rotations to meters and accounting for motor rotating
    // the gear
    @Deprecated
    public double getPosition() {
        return driveMotor.getPosition().getValueAsDouble() * 2 * Math.PI * Constants.Measurement.WHEELRADIUS
                / Constants.SwerveModule.GearRatio.DRIVE;
    }

    public double getEncoderPosition() {
        return (c_encoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI)
                - absolutePositionEncoderOffset;
    }

    /*
     * Conversion to get in meters/sec to rotations/sec and accounting for motor
     * rotating the gear
     */
    public double toRotationsPerSecond(SwerveModuleState state) {
        return state.speedMetersPerSecond * m_driveRotationsPerMeter;
    }

    public void stopDriving() {
        driveMotor.stopMotor();
    }

    BaseStatusSignal[] getSignals() {
        return m_signals;
    }
}
