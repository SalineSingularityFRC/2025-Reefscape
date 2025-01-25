package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Preferences;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.Command;


public class IntakeSubsystem extends SubsystemBase{
    private Spark rightMotor;
    private Spark leftMotor;
    private LaserCan intakeSensor;
    private LaserCan shooterSensor;
    private double motorSpeed;
    private double sensingDistance;

    public IntakeSubsystem(Spark lMotor, Spark rMotor, LaserCan intake, LaserCan shooter){
        rightMotor = rMotor;
        leftMotor = lMotor;
        intakeSensor = intake;
        shooterSensor = shooter;
        sensingDistance = Preferences.getDouble("Sensing Distance", 100);
        motorSpeed = Preferences.getDouble("Intake Motor Speed", 1);
    }

    public void periodic(){
        SmartDashboard.putNumber("Intake Sensor", getSensorValue(intakeSensor));
        SmartDashboard.putNumber("Shooter Sensor", getSensorValue(shooterSensor));
    }

    public boolean canSeeCoral(LaserCan sensor){
        if(sensor.getMeasurement() != null && getSensorValue(sensor) <= sensingDistance){
            return true;
        }
        return false;
    }

    public boolean coralInIntake(){
        return canSeeCoral(intakeSensor);
    }

    public boolean coralInShooter(){
        return canSeeCoral(shooterSensor);
    }

    public int getSensorValue(LaserCan sensor){
        return sensor.getMeasurement().distance_mm;
    }

    public Command runMotors(){
        return runEnd(
            () -> {
                leftMotor.set(-motorSpeed);
                rightMotor.set(-motorSpeed); },
             () -> {
                leftMotor.stopMotor();
                rightMotor.stopMotor();
            }
        );
    }

    public Command shootCoral(){
        return runMotors().until(() -> !coralInShooter());
    }

    public Command intakeCoral(){
        return runMotors().until(() -> coralInShooter());
    }
}
