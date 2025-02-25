package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class RumbleCommandStart extends Command{
    public CommandXboxController controller;

    public RumbleCommandStart(CommandXboxController xboxController) {
        controller = xboxController;
    }

    public void execute() {
        controller.getHID().setRumble(RumbleType.kLeftRumble, 0.5);
        controller.getHID().setRumble(RumbleType.kRightRumble, 0.5);
    }

    @Override
    public void end(boolean interrupted) {
        controller.getHID().setRumble(RumbleType.kLeftRumble, 0);
        controller.getHID().setRumble(RumbleType.kRightRumble, 0);
    }
}
