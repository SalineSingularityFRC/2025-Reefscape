// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  public Robot() {
    CanBridge.runTCP();
    DataLogManager.start();
    // addPeriodic(() -> {
    //   m_robotContainer.updateOdometry();
    // }, 0.01, 0.005);
  }

  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "2025-Reefscape");

    switch (Constants.Modes.currentMode) {
      case REAL: // on a real robot
        System.out.println("REAL!");
        Logger.addDataReceiver(new WPILOGWriter()); // logs to /logs/ on USB stick
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case SIM: // on "Simulate Robot Code"
        System.out.println("SIM!");
        Logger.addDataReceiver(new WPILOGWriter()); // logs to logs folder in project
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        System.out.println("REPLAY!");
        // setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();

    AlertManager.initialize();
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.updateOdometry();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {

    m_robotContainer.updateOdometry();

    m_robotContainer.updateRotationPIDSetpoint();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {

    m_robotContainer.updateOdometry();

    m_robotContainer.updateRotationPIDSetpoint();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
