package org.firstinspires.ftc.teamcode.Opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class TestNoPinPoint extends LinearOpMode {
//    public static TelemetryPacket packet = new TelemetryPacket();
//    FtcDashboard dash = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);

        ElapsedTime time = new ElapsedTime();
//        dash.sendTelemetryPacket(packet);

        waitForStart();
        time.reset();

        while (opModeIsActive()) {
            mecanumDrive.update();
            telemetry.addData("Loop_Time",time.seconds());
            telemetry.addData("X",mecanumDrive.getPoseEstimate().getX());
            telemetry.addData("Y",mecanumDrive.getPoseEstimate().getY());
            telemetry.addData("Heading",mecanumDrive.getPoseEstimate().getHeading());
            time.reset();
            telemetry.update();
//            dash.sendTelemetryPacket(packet);
//            packet = new TelemetryPacket();
        }
    }
}
