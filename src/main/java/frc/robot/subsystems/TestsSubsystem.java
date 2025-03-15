/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestsSubsystem extends SubsystemBase {
  private Servo ledController = new Servo(0);
  private double dc;
  final double FREQ = 1000;
  final int maxPeriodInUS = (int)(1.0e6/FREQ);
  final int minPeriodInUS = 0;
  final int middlePeriodInUS = (maxPeriodInUS - minPeriodInUS) / 2;

  /**
   * Creates a new ledController.
   */
  public TestsSubsystem() {
    // ledController.setPeriodMultiplier(PeriodMultiplier.k1X);
    // ledController.setBoundsMicroseconds(maxPeriodInUS, 0, middlePeriodInUS, 0, minPeriodInUS);
		ledController.setBoundsMicroseconds(1950, 1504, 1500, 1496, 1050); 
    setDutyCycle(1/FREQ);
  }

  @Override
  public void periodic() {
    dc = SmartDashboard.getNumber("Test/DC", dc);
    // double pulseWidthInSeconds = this.dc * (1.0/FREQ);
    // int pulseWidthInMicroSeconds = (int)(pulseWidthInSeconds * 1e6);
    // ledController.setPulseTimeMicroseconds(pulseWidthInMicroSeconds);
    ledController.set(dc);
    // SmartDashboard.putNumber("Test/PulseWidthUS", pulseWidthInMicroSeconds);
  }

  public void setDutyCycle(double dc) {
    this.dc = dc;
    SmartDashboard.putNumber("Test/DC", dc);
  }
}