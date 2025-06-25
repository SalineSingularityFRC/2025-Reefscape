package frc.robot.commands.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CustomOperatorController extends CommandXboxController{
    public CustomOperatorController(int port) {
        super(port);
    }

    public Trigger bargeScoring() {return a();}

    public Trigger feederStation() {return leftBumper();}
    public Trigger L2_Left() {return rightBumper();}
    public Trigger L3_Left() {return back();}
    public Trigger L4_Left() {return start();}
    public Trigger L2_Right() {return b();}
    public Trigger L3_Right() {return x();}
    public Trigger L4_Right() {return y();}

    public Trigger leftSource() {return button(11);}
    public Trigger rightSource() {return button(12);}

    public Trigger elevatorManualUp() {return axisGreaterThan(1, 0.1);}
    public Trigger elevatorManualDown() {return axisLessThan(1, -0.1);}

    public Trigger intakeCoral() {return rightStick();}
    public Trigger shootCoral() {return leftStick();}

    public Trigger fineAdjustLeft() {return axisGreaterThan(0, 0.1);}
    public Trigger fineAdjustRight() {return axisLessThan(0, -0.1);}
}
