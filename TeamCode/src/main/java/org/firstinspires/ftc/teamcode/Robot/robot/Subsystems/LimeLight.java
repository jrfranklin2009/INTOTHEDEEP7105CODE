package org.firstinspires.ftc.teamcode.Robot.robot.Subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.CommandFrameWork.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.robot.Input;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.PinPoint.PinPoint_Odo;

public class LimeLight extends Subsystem {

    Limelight3A limelight;

    DriveTrain odo;

    double x,y;

    public LimeLight(DriveTrain odo){
        this.odo = odo;
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
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
        limelight.updateRobotOrientation(odo.odo.getHeading());
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose_MT2();
                Dashboard.addData("Botpose", botpose.toString());
                converter(result);
                odo.mecanumDrive.setPoseEstimate(new Pose2d(x,y,botpose.getOrientation().getYaw()));
                odo.odo.setPosition(new Pose2D(INCH,x,y, AngleUnit.DEGREES,result.getBotpose().getOrientation().getYaw()));
            }
            }

    }

    public void converter(LLResult llResult){
        x = llResult.getBotpose().getPosition().x * -36.15384; //35.9375
        y = llResult.getBotpose().getPosition().y * -36.15384;

      //  -23 -47 rr cordinates
        //        .64 1.30 ll cordinates
    }

    @Override
    public void shutdown() {
        limelight.shutdown();
    }
}
