package frc.robot.subsystems;

import java.nio.ByteBuffer;

import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANMessageNotFoundException;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LidarOverCAN extends SubsystemBase {

    public LidarOverCAN() {
        super();
    }

    @Override
    public void periodic() {
        ByteBuffer targetedMessageID = ByteBuffer.allocateDirect(4); // Must be direct
        targetedMessageID.asIntBuffer().put(0, 0x1E040000);
        ByteBuffer timeStamp = ByteBuffer.allocateDirect(4); // Allocate memory for time stamp
        byte[] message = new byte[] {};
        try {
            // Return call is data, selection is assigned
            message = CANJNI.FRCNetCommCANSessionMuxReceiveMessage(targetedMessageID.asIntBuffer(),
                    0xFFFFFFFF, timeStamp);
        } catch (CANMessageNotFoundException e) {
            // Nothing
        } catch (Exception e) {
            // Other exception, print it out to make sure user sees it
            System.out.println(e.toString());
        }
        
        System.out.println("Last Can: " + message.toString());
        SmartDashboard.putRaw("Lidar Debug Last CAN", message);
    }
}
