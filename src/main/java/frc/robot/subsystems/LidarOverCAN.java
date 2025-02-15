package frc.robot.subsystems;

import java.nio.ByteBuffer;

import edu.wpi.first.hal.CANStreamMessage;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANMessageNotFoundException;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LidarOverCAN extends SubsystemBase {

    private int handle;
    
    
        public LidarOverCAN() {
            super();
            handle = CANJNI.openCANStreamSession(0x1E040000, 0xFFFFFFFF, 1000);
        
    }


    @Override
    public void periodic() {
        ByteBuffer targetedMessageID = ByteBuffer.allocateDirect(4); // Must be direct
        targetedMessageID.asIntBuffer().put(0, 0x8041481);
        ByteBuffer timeStamp = ByteBuffer.allocateDirect(4); // Allocate memory for time stamp
        byte[] message = new byte[] {};
        CANStreamMessage[] messages = new CANStreamMessage[4];
        try {
            // Return call is data, selection is assigned
            CANJNI.readCANStreamSession(handle, messages, 4);
        } catch (CANMessageNotFoundException e) {
            // Nothing
        } catch (Exception e) {
            // Other exception, print it out to make sure user sees it
            System.out.println(e.toString());
        }
        
        if (messages != null && messages.length > 0) {
            for( CANStreamMessage msg : messages) {
            //SmartDashboard.putRaw("Lidar Debug Last CAN", message);
            //System.out.println("Last Can: " + message.toString());
            System.out.println(String.format("%08X", msg.messageID));
            /*for (byte b : message) {
                System.out.print(String.format("%02X ", b));
            } */
            System.out.println(""); 
        }
        }
    }
}
