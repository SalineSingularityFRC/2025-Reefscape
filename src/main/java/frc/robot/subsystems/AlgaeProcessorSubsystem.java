package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeProcessorSubsystem extends SubsystemBase {
    private TalonFX intakeMotor;
    private final VelocityTorqueCurrentFOC intakeRequest, spitRequest;
    private Servo ledController = new Servo(1);
    private double dc;
    final double FREQ = 1000;
    final int maxPeriodInUS = (int)(1.0e6/FREQ);
    final int minPeriodInUS = 0;
    final int middlePeriodInUS = (maxPeriodInUS - minPeriodInUS) / 2;


    public AlgaeProcessorSubsystem() {
        intakeMotor = new TalonFX(Constants.CanId.Processor.INTAKE_MOTOR);
        intakeRequest = new VelocityTorqueCurrentFOC(Constants.Processor.intakeSpeed.getValue()).withSlot(0);
        spitRequest = new VelocityTorqueCurrentFOC(Constants.Processor.spitSpeed.getValue()).withSlot(0);

        ledController.setPeriodMultiplier(PeriodMultiplier.k1X);
        ledController.setBoundsMicroseconds(maxPeriodInUS, 0, middlePeriodInUS, 0, minPeriodInUS);
		ledController.setBoundsMicroseconds(1950, 1504, 1500, 1496, 1050); 
        setDutyCycle(1/FREQ);
    
    
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.Slot0.kP = Constants.Processor.kP.getValue();
        intakeMotor.getConfigurator().apply(intakeConfig);
    }
    @Override
    public void periodic() {
        intakeRequest.Velocity = Constants.Processor.intakeSpeed.getValue();
        spitRequest.Velocity = Constants.Processor.spitSpeed.getValue();    
   
      dc = SmartDashboard.getNumber("Test/DC", dc);
      double pulseWidthInSeconds = this.dc * (1.0/FREQ);
      int pulseWidthInMicroSeconds = (int)(pulseWidthInSeconds * 1e6);
      ledController.setPulseTimeMicroseconds(pulseWidthInMicroSeconds);
      ledController.set(dc);
      SmartDashboard.putNumber("Test/PulseWidthUS", pulseWidthInMicroSeconds);
    }

    public Command intakeProcessor() {
        return runEnd(() -> {
            intakeMotor.setControl(intakeRequest);
        }, () -> {
            intakeMotor.stopMotor();
        });
    }

    public Command spitProcessor() {
        return runEnd(() -> {
            intakeMotor.setControl(spitRequest);
        }, () -> {
            intakeMotor.stopMotor();
        });
    }
    
  public void setDutyCycle(double dc) {
    this.dc = dc;
    SmartDashboard.putNumber("Test/DC", dc);
    }
}