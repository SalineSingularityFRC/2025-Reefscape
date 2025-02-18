package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.hal.CANStreamMessage;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANMessageNotFoundException;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LidarOverCAN extends SubsystemBase {

    private int handle;
    // HashMap to store CAN message IDs (as strings with a "0x" prefix) and their occurrence counts
    private HashMap<String, Integer> idCountMap = new HashMap<>();

    public LidarOverCAN() {
        super();
        handle = CANJNI.openCANStreamSession(0x40000555, 0xFFFFFFF0, 1000);
    }

    @Override
    public void periodic() {
        CANStreamMessage[] messages = new CANStreamMessage[4];

        try {
            // Read up to 4 messages from the CAN stream
            CANJNI.readCANStreamSession(handle, messages, 4);
        } catch (CANMessageNotFoundException e) {
            // No messages found, do nothing
        } catch (Exception e) {
            // Print any other exception
            System.out.println(e.toString());
        }

        if (messages != null && messages.length > 0) {
            for (CANStreamMessage msg : messages) {
                if (msg == null) {
                    continue;
                }
                // Format the message ID as a hexadecimal string with "0x" prefix
                String idStr = String.format("0x%08X", msg.messageID);
                // Update the count for this message ID in the map
                idCountMap.put(idStr, idCountMap.getOrDefault(idStr, 0) + 1);
                for (byte b : msg.data){
                System.out.print(b + " ");
                }
                System.out.println();
            }
            // Print the entire hashmap after processing messages
            System.out.println("CAN ID Count Map: " + idCountMap.toString());
        }

    }
}
