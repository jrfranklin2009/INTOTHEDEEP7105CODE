package org.firstinspires.ftc.teamcode.Robot.robot.Subsystems;

import static com.ThermalEquilibrium.homeostasis.Utils.MathUtils.AngleWrap;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER;

import com.ThermalEquilibrium.homeostasis.Utils.MathUtils;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.MathUtil;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.CommandFrameWork.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DriveTrain.DriveTrain;

public class LimeLight extends Subsystem {

    Limelight3A limelight;

    DriveTrain odo;

    Pose3D botpose;

    LLResult result;

    public static double x,y, heading;

    public static double robotx,roboty,robotH;

    public LimeLight(DriveTrain odo){
        this.odo = odo;
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        x = 0;
        y = 0;

        limelight = hwMap.get(Limelight3A.class,"limelight");

        limelight.pipelineSwitch(0);

        limelight.setPollRateHz(100);
        /*
         * Starts polling for data.
         */
        limelight.start();
    }

    @Override
    public void periodicAuto() {
        result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                botpose = result.getBotpose_MT2();
//                limelight.updateRobotOrientation(odo.getHeading());
//                converter();
                odo.mecanumDrive.setPoseEstimate(new Pose2d(getBotX(), getBotY(), getBotHeading()));
//                odo.odo.setPosition(new Pose2D(INCH, getBotX(), getBotY(), AngleUnit.DEGREES, getBotHeading()));
            }
        }
    }

    public void converter(){
        x = (getBotX() * -7.8692); //35.9375
        y = (getBotY() * -37.4898);
//        heading = AngleWrap(getBotHeading(),180);
      //  -23 -47 rr cordinates
        //        .64 1.30 ll cordinates
    }

    public double AngleWrap(double pos,double offset){
        return MathUtils.AngleWrap(pos+offset);
    }

    public double getBotHeading(){
        return botpose.getOrientation().getYaw();
    }

    public double getBotX(){
        return botpose.getPosition().x;
    }

    public double getBotY(){
        return botpose.getPosition().y;
    }

//    public void setOdo(double x,double y, double heading){
//        robotx = x + odo.getXPos();
//        roboty = y + odo.getYPos();
//        robotH = heading + odo.getHeading();
//    }

    @Override
    public void shutdown() {
        limelight.shutdown();
    }
}
