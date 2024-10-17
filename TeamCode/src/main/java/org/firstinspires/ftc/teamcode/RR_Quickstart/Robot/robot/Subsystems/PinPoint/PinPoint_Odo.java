package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.PinPoint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Subsystem;
import org.firstinspires.ftc.teamcode.RR_Quickstart.drive.PinPoint_MecanumDrive;

public class PinPoint_Odo extends Subsystem {

    GoBildaPinpointDriver odo;

    PinPoint_MecanumDrive mecanumDrive;

    Telemetry telemetry;

    public PinPoint_Odo (Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        odo = hwMap.get(GoBildaPinpointDriver.class,"pinpointodo");
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setEncoderResolution(GoBildaPinpointDriver.OdometryPods.opTii_ODOMETRY);
        odo.setOffsets(5,0);
        this.mecanumDrive = new PinPoint_MecanumDrive(hwMap,odo);
    }

    @Override
    public void periodicAuto() {
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
