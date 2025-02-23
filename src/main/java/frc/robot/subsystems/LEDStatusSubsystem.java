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

public class LEDStatusSubsystem extends SubsystemBase {
  private double currentColorPWMValue = 0.0;
  private VictorSP ledController = new VictorSP(0);

  /**
   * Creates a new ledController.
   */
  public LEDStatusSubsystem() {
    setColor(LEDColor.BLUE);
  }

  @Override
  public void periodic() {
    ledController.set(currentColorPWMValue);
    SmartDashboard.putNumber("LED/colorValue", currentColorPWMValue);
  }

  static final Map<LEDColor, Double> colorToPwmMap = Map.of(
    LEDColor.ORANGE, 0.65,
    LEDColor.YELLOW, 0.69,
    LEDColor.RED, 0.61,
    LEDColor.GREEN, 0.77,
    LEDColor.BLUE, 0.87,
    LEDColor.VIOLET, 0.91
  );

  public void setColor(LEDColor color) {
    SmartDashboard.putString("LED/colorName", color.name());
    Double colorPWMValue = colorToPwmMap.get(color);
    if (colorPWMValue != null) {
      currentColorPWMValue = colorPWMValue;
    }
  }
}