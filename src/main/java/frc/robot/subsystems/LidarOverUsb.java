package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LidarOverUsb extends SubsystemBase {
    private SerialPort serial;

    public LidarOverUsb() {
        super();

        serial = new SerialPort(115200, Port.kUSB);
        serial.enableTermination();
        serial.setTimeout(1);
        serial.setFlowControl(SerialPort.FlowControl.kNone);
        serial.setReadBufferSize(32);
        serial.setWriteBufferSize(8);
        serial.setWriteBufferMode(SerialPort.WriteBufferMode.kFlushOnAccess);
    }

    @Override
    public void periodic() {
        super.periodic();
        String data = serial.readString();
        System.out.print("Serial ");
        System.out.println(data);

        SmartDashboard.putString("Lidar Debug Last Received", data);
    }
}
