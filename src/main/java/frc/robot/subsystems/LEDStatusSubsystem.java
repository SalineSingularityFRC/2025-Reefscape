/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;

public class LEDStatusSubsystem extends SubsystemBase {
  private double currentColorPWMValue = 0.0;
  private VictorSP ledController = new VictorSP(0);
  private final IntakeSubsystem intake;
  private ElevatorSubsystem elevator;

  /**
   * Creates a new ledController.
   */
  public LEDStatusSubsystem(IntakeSubsystem intake, ElevatorSubsystem elevator) {
    this.intake = intake;
    this.elevator = elevator;
    setColor(LEDColor.BLUE);
  }

  @Override
  public void periodic() {
    checkStatus();
    ledController.set(currentColorPWMValue);
    // SmartDashboard.putNumber("LED/colorValue", currentColorPWMValue);
    // ledController.set(frc.robot.Constants.LED.PWM_VALUE.getValue());
  }

  static final Map<LEDColor, Double> colorToPwmMap = Map.of(
      LEDColor.ORANGE, 0.59, // good
      LEDColor.GOLD, 0.67, // unfinished magenta
      LEDColor.RED, 0.61,// unfinished orange ish
      LEDColor.GREEN, 0.87, // good 
      LEDColor.BLUE, 0.75, // good
      LEDColor.VIOLET, 0.71, // good
      LEDColor.WHITE, 0.91,
      LEDColor.FLASHBLUE, 0.15); // Unfinished White

  public void setColor(LEDColor color) {
    SmartDashboard.putString("LED/colorName", color.name());
    Double colorPWMValue = colorToPwmMap.get(color);
    if (colorPWMValue != null) {
      currentColorPWMValue = colorPWMValue;
    }
  }

  private void checkStatus() {

    //Orange for LOADING(Coral in intake, not in position and ready), Violet for LOADED(all variables true)
    boolean isLoaded = false;
    boolean isLoading = false;
    boolean isReadyToShoot = false;
    boolean isIntaking = false;

    if (intake.coralInIntake()) {
      isLoading = true;
    }

    if (intake.isMotorRunning() && intake.noCoralDetected()) {
      isIntaking = true;
    }

    if (elevator.isAtSetpoint() && intake.readyToShoot()) {
      isReadyToShoot = true;
    }

    if (elevator.isElevatorAtSetpoint() && intake.readyToShoot()) {
      isLoaded = true;
    }

    if (isLoading) {
      setColor(LEDColor.ORANGE);
    } else if (isReadyToShoot) {
      setColor(LEDColor.VIOLET);
    } else if (isLoaded) {
      setColor(LEDColor.WHITE);
    } else if (isIntaking) {
      setColor(LEDColor.FLASHBLUE);
    } else {
      setColor(LEDColor.GREEN);
    }
  } 
}