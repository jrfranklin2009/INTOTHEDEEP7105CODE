package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.drive.PinPoint_MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    GoBildaPinpointDriver odo;
    @Override
    public void runOpMode() throws InterruptedException {
        PinPoint_MecanumDrive drive = new PinPoint_MecanumDrive(hardwareMap, odo);
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpointodo");
        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }
}
