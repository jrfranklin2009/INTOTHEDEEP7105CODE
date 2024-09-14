package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Subsystem;

public class Dashboard extends Subsystem {
    public static TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dash = FtcDashboard.getInstance();
//    public static Telemetry telemetry;
//    static OpMode opMode;
    ElapsedTime time = new ElapsedTime();

    @Override
    public void initAuto(HardwareMap hwMap) {
        dash.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }
    @Override
    public void periodicAuto() {
        dash.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();

//        packet.put("Loop Time",time.seconds());
        addData("Loop Time",time.seconds());
        time.reset();
    }
    @Override
    public void shutdown() {

    }
    public static void addData(String string, Object object){
        packet.put(string,object);
//        opMode.telemetry.addData(string,object);
//        opMode.updateTelemetry(opMode.telemetry);
    }
    public static void addLine(String string){
        packet.addLine(string);
//        opMode.telemetry.addLine(string);
//        opMode.updateTelemetry(opMode.telemetry);
    }


}
