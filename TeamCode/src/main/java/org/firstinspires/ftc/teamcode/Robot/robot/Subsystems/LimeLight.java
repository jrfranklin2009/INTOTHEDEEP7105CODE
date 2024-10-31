package org.firstinspires.ftc.teamcode.Robot.robot.Subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.CommandFrameWork.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.PinPoint.PinPoint_Odo;

public class LimeLight extends Subsystem {

    Limelight3A limelight;

    DriveTrain odo;

    public LimeLight(DriveTrain odo){
        this.odo = odo;
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class,"limelight");

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.
         */
        limelight.start();
    }

    @Override
    public void periodicAuto() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                Dashboard.addData("tx", result.getTx());
                Dashboard.addData("ty", result.getTy());
                Dashboard.addData("Botpose", botpose.toString());
//                odo.mecanumDrive.setPoseEstimate(new Pose2d(result.getBotpose().getPosition().x,result.getBotpose().getPosition().y,result.getBotpose().getOrientation().getYaw()));
//                odo.odo.setPosition(new Pose2D(METER,result.getBotpose().getPosition().x,result.getBotpose().getPosition().y, AngleUnit.DEGREES,result.getBotpose().getOrientation().getYaw()));
            }
        }

    }
    @Override
    public void shutdown() {
        limelight.shutdown();
    }
}
