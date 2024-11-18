package org.firstinspires.ftc.teamcode.Opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Dashboard;

@TeleOp
public class TestPinPoint extends LinearOpMode {
//    public static TelemetryPacket packet = new TelemetryPacket();
//    FtcDashboard dash = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {

        GoBildaPinpointDriver odo;

        ElapsedTime time = new ElapsedTime();

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpointodo");
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.optii);
        odo.setOffsets(171.25336,0);
//        dash.sendTelemetryPacket(packet);
//        resetPosAndHeading();

        waitForStart();
        time.reset();
        odo.resetPosAndIMU();

        while (opModeIsActive()){
            odo.update();
            telemetry.addData("Loop_Time",time.seconds());
            telemetry.addData("DeviceStatus",odo.getDeviceStatus());
            telemetry.addData("PinLoop_Time",odo.getLoopTime());
            telemetry.addData("X_Pos",odo.getPosX());
            telemetry.addData("Y_Pos",odo.getPosY());
            telemetry.addData("Heading",odo.getHeading());
            time.reset();
            telemetry.update();
//            dash.sendTelemetryPacket(packet);
//            packet = new TelemetryPacket();
        }
    }
}
