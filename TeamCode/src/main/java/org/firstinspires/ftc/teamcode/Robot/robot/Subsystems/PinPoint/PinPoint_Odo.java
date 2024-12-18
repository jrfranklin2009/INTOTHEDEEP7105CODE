package org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.PinPoint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.CommandFrameWork.Subsystem;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.drive.PinPoint_MecanumDrive;

public class PinPoint_Odo extends Subsystem {

    GoBildaPinpointDriver odo;

    public PinPoint_MecanumDrive mecanumDrive;

    private final static int CPS_STEP = 0x10000;

    private double[] velocityEstimates;

//    Telemetry telemetry;

    public PinPoint_Odo () {
//        this.telemetry = telemetry;
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        odo = hwMap.get(GoBildaPinpointDriver.class,"pinpointodo");
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.optii);
        odo.setOffsets(171.25336,0);
        resetPosAndHeading();
//        this.mecanumDrive = new PinPoint_MecanumDrive(hwMap,od);
    }

    @Override
    public void periodicAuto() {
        Dashboard.addData("Status", odo.getDeviceStatus());
        Dashboard.addData("Pinpoint Frequency", odo.getFrequency());
        Dashboard.addData("X_Pos",odo.getPosX());
        Dashboard.addData("Y_Pos",odo.getPosY());
        Dashboard.addData("Heading",Math.toDegrees(odo.getHeading()));
        Dashboard.addData("Heading_Radians?",odo.getHeading());
        mecanumDrive.update();
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

    public double getCorrectedVelocity(double input) {
        double median = velocityEstimates[0] > velocityEstimates[1]
                ? Math.max(velocityEstimates[1], Math.min(velocityEstimates[0], velocityEstimates[2]))
                : Math.max(velocityEstimates[0], Math.min(velocityEstimates[1], velocityEstimates[2]));
        return inverseOverflow(input, median);
    }
//
    private static double inverseOverflow(double input, double estimate) {
        // convert to uint16
        int real = (int) input & 0xffff;
        // initial, modulo-based correction: it can recover the remainder of 5 of the upper 16 bits
        // because the velocity is always a multiple of 20 cps due to Expansion Hub's 50ms measurement window
        real += ((real % 20) / 4) * CPS_STEP;
        // estimate-based correction: it finds the nearest multiple of 5 to correct the upper bits by
        real += Math.round((estimate - real) / (5 * CPS_STEP)) * 5 * CPS_STEP;
        return real;
    }

}
