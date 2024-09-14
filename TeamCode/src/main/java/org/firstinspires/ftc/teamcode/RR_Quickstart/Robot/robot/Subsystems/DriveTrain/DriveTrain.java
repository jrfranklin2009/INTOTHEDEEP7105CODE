package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DriveTrain;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Subsystem;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Input;
import org.firstinspires.ftc.teamcode.RR_Quickstart.drive.SampleMecanumDrive;

@Config
public class DriveTrain extends Subsystem {
   public SampleMecanumDrive mecanumDrive;
   public static double headingP = 1;
   public static double xyP = 1;

   DriveSpeed driveSpeed = DriveSpeed.Fast;

   boolean slow = false;

   public DriveTrain(HardwareMap hwMap){
       this.mecanumDrive = new SampleMecanumDrive(hwMap);
   }

    @Override
    public void initAuto(HardwareMap hwMap) {
       mecanumDrive = new SampleMecanumDrive(hwMap);
    }

    @Override
    public void initTeleop(HardwareMap hwMap) {
        mecanumDrive = new SampleMecanumDrive(hwMap);
    }

    @Override
    public void periodicAuto() {

    }

    @Override
    public void shutdown() {
        mecanumDrive.setMotorPowers(0,0,0,0);
    }

    public void setMotorPower(float x) {
       mecanumDrive.setMotorPowers(x,x,x,x);
    }
    public void RobotRelative(Input input){
       if (driveSpeed == DriveSpeed.Fast) {
           mecanumDrive.setWeightedDrivePower(new Pose2d(
                   -input.getLeft_stick_y(),
                   -input.getLeft_stick_x(),
                   -input.getRight_stick_x()));
       } else if (driveSpeed == DriveSpeed.Slow) {
           mecanumDrive.setWeightedDrivePower(new Pose2d(
                   (-input.getLeft_stick_y() * .3),
                   (-input.getLeft_stick_x() * .3),
                   (-input.getRight_stick_x() * .3)));
           }
       }
    public void setStates(Input input){
       if (input.isCrossPressed() && slow){
           slow = false;
       } else if (input.isCrossPressed() && !slow) {
           slow = true;
       } else if (slow) {
           driveSpeed = DriveSpeed.Slow;
       } else if (!slow) {
           driveSpeed = DriveSpeed.Fast;
       }
    }

    public void setStatesJohn(Input input){
       if (input.isLeft_bumper()) {
            driveSpeed = DriveSpeed.Slow;
        } else if (!input.isLeft_bumper()) {
            driveSpeed = DriveSpeed.Fast;
        }
    }

    public void RobotRelativeSlow(Input input){
       mecanumDrive.setWeightedDrivePower(new Pose2d(
               0.2*-input.getLeft_stick_y(),
               0.2*-input.getLeft_stick_x(), 0.2*-input.getRight_stick_x()));
    }
    public void followTrajectoryAsync(Trajectory trajectory){
       mecanumDrive.followTrajectoryAsync(trajectory);
    }

    public void followTrajectory(Trajectory trajectory){
        mecanumDrive.followTrajectory(trajectory);
    }

    public void turn(double angle){
       mecanumDrive.turn(angle);
    }

    public Pose2d poseEstimate(){
        return  mecanumDrive.getPoseEstimate();
    }

    public void setPoseEstimate(Pose2d pose){
       mecanumDrive.setPoseEstimate(pose);
    }

    public void update(){
       mecanumDrive.update();
    }

//    public void updateHeadingIMU(){
//       mecanumDrive.setPoseEstimate(new Pose2d(mecanumDrive.getPoseEstimate().getX(),mecanumDrive.getPoseEstimate().getY(),Math.toRadians(heading)));
//    }


    public void lockPosition (Pose2d target){
       Pose2d difference = target.minus(poseEstimate());
       Vector2d xy = difference.vec().rotated(-poseEstimate().getHeading());

       double heading = Angle.normDelta(target.getHeading()) - Angle.normDelta(poseEstimate().getHeading());
       mecanumDrive.setWeightedDrivePower(new Pose2d(xy.times(xyP),heading * headingP));
    }

//    public void updateHeading(double offset){
//       heading = imu.getHeading() + offset;
//    }
//
//    public double resetIMU(){
//        imu.reset();
//        heading = imu.getHeading();
////        heading = heading + offset;
//        return heading;
//    }

    public enum DriveSpeed{
       Fast,
        Slow
    }
}
