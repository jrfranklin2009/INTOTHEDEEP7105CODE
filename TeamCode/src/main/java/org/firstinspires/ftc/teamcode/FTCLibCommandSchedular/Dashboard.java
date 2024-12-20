package org.firstinspires.ftc.teamcode.FTCLibCommandSchedular;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Dashboard extends BetterSubsystems{

    public static TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dash = FtcDashboard.getInstance();
    //    public static Telemetry telemetry;
//    static OpMode opMode;
    ElapsedTime time = new ElapsedTime();

    @Override
    public void init() {

    }

    @Override
    public void periodic() {

    }
}
