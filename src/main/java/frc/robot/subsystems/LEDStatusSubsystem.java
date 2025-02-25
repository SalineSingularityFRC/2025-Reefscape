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
import frc.robot.Limelight;
import frc.robot.Constants.Elevator;

public class LEDStatusSubsystem extends SubsystemBase {
  private double currentColorPWMValue = 0.0;
  private VictorSP ledController = new VictorSP(0);
  private final IntakeSubsystem intake;
  private frc.robot.Limelight limelight;
  private ElevatorSubsystem elevator;

  /**
   * Creates a new ledController.
   */
  public LEDStatusSubsystem(IntakeSubsystem intake, Limelight limeLight, ElevatorSubsystem elevator) {
    this.intake = intake;
    this.limelight = limeLight;
    this.elevator = elevator;
    setColor(LEDColor.BLUE);
  }

  @Override
  public void periodic() {
    checkStatus();
    ledController.set(currentColorPWMValue);
    SmartDashboard.putNumber("LED/colorValue", currentColorPWMValue);
  }

  static final Map<LEDColor, Double> colorToPwmMap = Map.of(
      LEDColor.ORANGE, 0.65,
      LEDColor.GOLD, 0.67,
      LEDColor.RED, 0.61,
      LEDColor.GREEN, 0.77,
      LEDColor.BLUE, 0.87,
      LEDColor.VIOLET, 0.91);

  public void setColor(LEDColor color) {
    SmartDashboard.putString("LED/colorName", color.name());
    Double colorPWMValue = colorToPwmMap.get(color);
    if (colorPWMValue != null) {
      currentColorPWMValue = colorPWMValue;
    }
  }

  private void checkStatus() {

    //Gold for LOADING(Coral in intake, not in position and ready), Violet for LOADED(all variables true)
    boolean commErrors = false;
    boolean isLoading = false;
    boolean isLoaded = false;

    if (intake.coralInIntake()) {
      isLoading = true;
    }

    if (elevator.isAtSetpoint() && intake.readyToShoot()) {
      isLoaded = true;
    }

    if (isLoading) {
      setColor(LEDColor.GOLD);
    } else if (isLoaded) {
      setColor(LEDColor.VIOLET);
    } else {
      setColor(LEDColor.GREEN);
    }
  } 
}