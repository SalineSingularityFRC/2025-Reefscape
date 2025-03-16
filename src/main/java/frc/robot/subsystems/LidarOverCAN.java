package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.hal.CANStreamMessage;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANMessageNotFoundException;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LidarOverCAN extends SubsystemBase {
    private int handle;
    private double[][] lidarData = new double[8][8]; // Assuming each message contains 8 bytes of data
    private boolean[] isDataValid = new boolean[8]; // Assuming each message contains 8 bytes of data
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

                int index = msg.messageID - 0x40000550;
                if (index >= 0 && index < 8) {
                    isDataValid[index] = true;
                    for (int i = 0; i < msg.data.length; i++) {
                        int measurement = (msg.data[i] & 0xFF);
                        measurement = measurement << 2;
                        measurement++;
                        lidarData[index][i] = (double) filterMeasurement(measurement);
                        if (lidarData[index][i] == 0) {
                            // isDataValid[index] = false;
                        }
                    }
                }
            }

            // if (loopIndex >= 25) {
            //     // Print the 2D array
            //     System.out.println("Lidar 2D Array:");
            //     for (int i = 0; i < lidarData.length; i++) {
            //         for (int j = 0; j < lidarData[i].length; j++) {
            //             System.out.print(lidarData[i][j] + " ");
            //         }
            //         System.out.println();
            //     }
            //     loopIndex = 0;
            // }

            // loopIndex++;
            // if (loopIndex % 20 == 0)
            // System.out.println("CAN ID Count Map: " + idCountMap.toString());
        }

        for (int row = 0; row < lidarData.length; row++) {

            if (isDataValid[row]) {
                double angle = OLSLineFit.compute(lidarData[row]);
                for (int j = 0; j < lidarData[row].length; j++) {
                    System.out.print(lidarData[row][j] + " ");
                }
                System.out.print(" -> " + angle);
                System.out.println();
            }
        }
    }

    int filterMeasurement(int measurementMM) {
        if (measurementMM < 30) {
            return 0;
        }

        if (measurementMM > 800) {
            return 0;
        }

        return measurementMM;
    }
}

class OLSLineFit {

    // Convert polar coordinates to Cartesian
    public static double[] polarToCartesian(double angle, double distance) {
        double x = distance * Math.cos(Math.toRadians(angle));
        double y = distance * Math.sin(Math.toRadians(angle));
        return new double[] { x, y };
    }

    // Compute Ordinary Least Squares (OLS) best-fit line: y = mx + b
    public static double computeOLSAngle(List<double[]> points) {
        int n = points.size();
        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;

        for (double[] p : points) {
            double x = p[0];
            double y = p[1];
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumX2 += x * x;
        }

        double meanX = sumX / n;
        double meanY = sumY / n;

        double m = (sumXY - n * meanX * meanY) / (sumX2 - n * meanX * meanX);

        // Convert slope (m) to angle in degrees
        double angle = Math.toDegrees(Math.atan(m));

        // Ensure heading is relative to LIDAR source
        // if (sumX < 0) { 
        //     angle += 180; // Adjust angle if line extends into negative x direction
        // }

        return angle;
    }

    // Compute Ordinary Least Squares (OLS) best-fit line: y = mx + b
    public static double[] computeOLSLine(List<double[]> points) {
        int n = points.size();
        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;

        for (double[] p : points) {
            double x = p[0];
            double y = p[1];
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumX2 += x * x;
        }

        double meanX = sumX / n;
        double meanY = sumY / n;

        double m = (sumXY - n * meanX * meanY) / (sumX2 - n * meanX * meanX);
        double b = meanY - m * meanX;

        return new double[] { m, b }; // y = mx + b
    }

    public static double compute(double[] distances) {
        double[] angles = { 1, 2, 3, 4, 5, 6, 7, 8 };

        List<double[]> points = new ArrayList<>();

        for (int i = 0; i < angles.length; i++) {
            if (distances[i] > 0) { 
                points.add(polarToCartesian(angles[i], distances[i]));
            }
        }

        double angle = computeOLSAngle(points);

        return angle;
    }
}
