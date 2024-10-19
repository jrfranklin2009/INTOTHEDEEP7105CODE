package org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.PinPoint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.CommandFrameWork.Subsystem;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver_Sus_Maybe;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.drive.PinPoint_MecanumDrive;

public class PinPoint_Odo extends Subsystem {

    GoBildaPinpointDriver_Sus_Maybe odo;

    PinPoint_MecanumDrive mecanumDrive;

    Telemetry telemetry;

    public PinPoint_Odo (Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        odo = hwMap.get(GoBildaPinpointDriver_Sus_Maybe.class,"pinpointodo");
        odo.setEncoderDirections(GoBildaPinpointDriver_Sus_Maybe.EncoderDirection.FORWARD, GoBildaPinpointDriver_Sus_Maybe.EncoderDirection.FORWARD);
        odo.setEncoderResolution(GoBildaPinpointDriver_Sus_Maybe.OdometryPods.opTii_ODOMETRY);
        odo.setOffsets(171.25336,0);
        this.mecanumDrive = new PinPoint_MecanumDrive(hwMap,odo);
    }

    @Override
    public void periodicAuto() {
        Dashboard.addData("Status", odo.getDeviceStatus());
        Dashboard.addData("Pinpoint Frequency", odo.getFrequency());
        Dashboard.addData("X_Pos",odo.getPosX());
        Dashboard.addData("Y_Pos",odo.getPosY());
        Dashboard.addData("Heading",Math.toRadians(odo.getHeading()));
        telemetry.addData("Status", odo.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", odo.getFrequency());
        telemetry.addData("X_Pos",odo.getPosX());
        telemetry.addData("Y_Pos",odo.getPosY());
        telemetry.addData("Heading",Math.toRadians(odo.getHeading()));
        telemetry.update();
        odo.update();
    }

    @Override
    public void shutdown() {

    }

    public void setPos(Pose2D pos){
        odo.setPosition(pos);
    }

    public void resetHeading(){
        odo.recalibrateIMU();
    }

    public void resetPosAndHeading(){
        odo.resetPosAndIMU();
    }

}
