package frc.robot.SwerveClasses;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.Constants;
import frc.robot.PID;

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

  private final PID drive_controller_gains = Constants.PidGains.SwerveModule.DRIVE_PID_CONTROLLER;
  private final PIDController m_drivePIDController = new PIDController(
      drive_controller_gains.P, drive_controller_gains.I, drive_controller_gains.D);

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

  }

  public SwerveModuleState getState(){

    return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble() * 2 * Math.PI * Constants.Measurement.WHEELRADIUS
        / Constants.MotorGearRatio.DRIVE, new Rotation2d(getEncoderPosition()));
  }

  protected void coast() {
    //driveMotor.set(0); // this is for when the joystick is not being moved at all
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
    angleMotor.setZeroAngle(getEncoderPosition());
  }

  public double getAngleClamped(){
    return angleMotor.getAngleClamped();
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getEncoderPosition()));
    state.cosineScale(new Rotation2d(getEncoderPosition()));
    double driveOutput = m_drivePIDController.calculate(driveMotor.get(), state.speedMetersPerSecond);

    switch(angleMotor.setAngle(state.angle.getRadians())){
      case Positive:
          driveMotor.set(driveOutput);
          break;
      case Negative:
        driveMotor.set(-driveOutput);
        break;
      default:
        break;
    }
  }

  public double getEncoderPosition() {
    return (c_encoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI)
        - absolutePositionEncoderOffset;
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
  public double getPosition() {
    return driveMotor.getPosition().getValueAsDouble() * 2 * Math.PI * Constants.Measurement.WHEELRADIUS
        / Constants.MotorGearRatio.DRIVE;
  }

  public void stopDriving() {
      driveMotor.stopMotor();
  }
}
