package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.hal.CANStreamMessage;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANMessageNotFoundException;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LidarOverCAN extends SubsystemBase {

    private int handle;
    private HashMap<String, Integer> idCountMap = new HashMap<>();
    private int[][] lidarData = new int[8][8]; // Assuming each message contains 8 bytes of data
    int loopIndex = 0;

    public LidarOverCAN() {
        super();
        handle = CANJNI.openCANStreamSession(0x40000555, 0xFFFFFFF0, 1000);
    }

    @Override
    public void periodic() {
        CANStreamMessage[] messages = new CANStreamMessage[8];

        try {
            CANJNI.readCANStreamSession(handle, messages, 8);
        } catch (CANMessageNotFoundException e) {
            // No messages found, do nothing
        } catch (Exception e) {
            System.out.println(e.toString());
        }

        if (messages != null && messages.length > 0) {
            for (CANStreamMessage msg : messages) {
                if (msg == null) {
                    continue;
                }
                String idStr = String.format("0x%08X", msg.messageID);
                idCountMap.put(idStr, idCountMap.getOrDefault(idStr, 0) + 1);

                int index = msg.messageID - 0x40000550;
                if (index >= 0 && index < 8) {
                    for(int i = 0; i < msg.data.length; i++){
                        int measurement = (msg.data[i] & 0xFF);
                        measurement = measurement<<2;
                        measurement++;
                        lidarData[index][i] = measurement;
                    }
                }
            }

            if (loopIndex >= 25) {
                // Print the 2D array
                System.out.println("Lidar 2D Array:");
                for (int i = 0; i < lidarData.length; i++) {
                    for (int j = 0; j < lidarData[i].length; j++) {
                        System.out.print(lidarData[i][j] + " ");
                    }
                    System.out.println();
                }
                loopIndex = 0;
            }

            loopIndex++;
            // if (loopIndex % 20 == 0)
            //     System.out.println("CAN ID Count Map: " + idCountMap.toString());
        }
    }
}
