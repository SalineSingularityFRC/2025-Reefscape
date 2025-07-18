package frc.robot.SwerveClasses;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.PID;

/**
 * This class owns a single Swerve Module's angle motor and is responsible for driving that motor to
 * a given angle
 */
public class SwerveAngle {
  /*
   * We need to have two class variables, a Falcon motor that we can use to control the angle of the module
   * and a variable for the zero poisition of the motor in radians (what value we need for it to be straight forward)
   */
  private double zeroPositionOffset;
  private TalonFX angleMotor;
  private PositionVoltage positionTarget;
  private PositionTorqueCurrentFOC positionTorqueCurrentFOC;

  /*
   * Our constructor needs to take a parameter that determines which CAN ID the falcon we are using has
   * and it needs to initialize the falcon motor and configure it (things like PID values and such)
   */

  protected SwerveAngle(int angleMotorId, String canNetwork, Boolean isInverted) {
    angleMotor = new TalonFX(angleMotorId, canNetwork);
    MotorOutputConfigs configs = new MotorOutputConfigs();
    
    // Factory reset before applying configs
    angleMotor.getConfigurator().apply(new TalonFXConfiguration());

    zeroPositionOffset = 0;

    positionTarget = new PositionVoltage(0).withSlot(0).withFeedForward(0.2);
    
    PID turnPID = Constants.PidGains.SwerveModule.TURNING_PID_CONTROLLER;
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = turnPID.P;
    slot0Configs.kI = turnPID.I;
    slot0Configs.kD = turnPID.D;;
    slot0Configs.kS = turnPID.S;

    // Initializing FOC control
    positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0);
    positionTorqueCurrentFOC.withSlot(0);
    positionTorqueCurrentFOC.withFeedForward(0);

    configs.NeutralMode = NeutralModeValue.Brake;
    angleMotor.getConfigurator().apply(slot0Configs);
    angleMotor.getConfigurator().apply(configs);
    
    angleMotor.setInverted(isInverted);
  }

  /*
   * This enum provides the current state of the angle motor, we should either be matching the requested angle (Positive),
   * 180° off of it (Negative), or in the process of getting to one of those positions (Moving)
   */
  protected enum AnglePosition {
    Positive,
    Negative,
    Moving
  }

  /*
   * This function takes an target angle (in radians) that we want the wheel to turn to
   * and updates the target position within the falcon motor so that it will move there.
   * It returns a value that indicates if we are in the target position, or 180° off of it,
   * or still working to get to one of those positions
   */
  protected AnglePosition setAngle(double targetAngle, String name) {
    double wheelPosition =
        getAngleClamped(); // the angle to set the wheel to minus the leftover full rotations
    double remainderRotations =
        getRemainderRotations(); // the additional rotations leftover from the wheel position
    double delta = wheelPosition - targetAngle;

    // This if else statement gets the closest angle to go to (absolute value)
    if (delta > Math.PI) {
      targetAngle += (2 * Math.PI);
    } else if (delta < -Math.PI) {
      targetAngle -= (2 * Math.PI);
    }

    // Recalculate the difference between the current angle according to the gyro and the target
    // angle according to the robot.
    delta = wheelPosition - targetAngle;
    AnglePosition currentPosition;

    currentPosition = AnglePosition.Positive;

    targetAngle += remainderRotations;
    targetAngle += zeroPositionOffset;

    SmartDashboard.putNumber("Target Angle" + name, Constants.SwerveModule.GearRatio.ANGLE * (targetAngle / (2 * Math.PI)));
    SmartDashboard.putNumber("Current Angle" + name, angleMotor.getPosition().getValueAsDouble());


    // Set the desired torque using FOC
    // angleMotor.setControl(
    //   positionTorqueCurrentFOC.withPosition(
    //     Constants.MotorGearRatio.ANGLE * (targetAngle / (2 * Math.PI))));

    // Let's drive
    angleMotor.setControl(
        positionTarget.withPosition(
            Constants.SwerveModule.GearRatio.ANGLE * (targetAngle / (2 * Math.PI))).withEnableFOC(true));

    if (Math.abs(delta % Math.PI) > Constants.AngleInaccuracy.MAX
        && Math.abs(delta % Math.PI) < Math.PI - Constants.AngleInaccuracy.MAX) {
      return AnglePosition.Moving; // Wheel is still in the process of turning
    }
    return currentPosition;
  }

  /*
   * Returns the angle (in radians) that the Talon is currently reporting we are in
   * minus our offset
   */
  protected double getAngle() {
    double talonRadians = (angleMotor.getPosition().getValueAsDouble() * 2 * Math.PI);
    double wheelRadians = talonRadians / Constants.SwerveModule.GearRatio.ANGLE;
    return wheelRadians - zeroPositionOffset;
  }

  /*
   * for getAngleClamped, return a value between [0, 2pi) that the talon says we are.
   * copied and pasted from the first lines of setAngle()
   */
  protected double getAngleClamped() {
    if (getAngle() >= 0) {
      return getAngle() % (2 * Math.PI);
    } else {
      return (2 * Math.PI) - (Math.abs(getAngle()) % (2 * Math.PI));
    }
  }

  /*
   * Returns the number of rotations in either direction the Talon is currently spun
   */
  private double getRemainderRotations() {
    return getAngle() - getAngleClamped();
  }

  /*
   * Set the zero angle based on the current angle (in radians) that we are reading from an external source.
   */
  protected void setZeroAngle(double currentAngle) {
    zeroPositionOffset += getAngle() - currentAngle;
  }
}
