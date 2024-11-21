package org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DriveTrain;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.CommandFrameWork.Subsystem;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Robot.robot.Input;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.drive.PinPoint_MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Config
public class DriveTrain extends Subsystem {
//   public SampleMecanumDrive mecanumDrive;
   public static double headingP = 1;
   public static double xyP = 1;

    public GoBildaPinpointDriver odo;

    public PinPoint_MecanumDrive mecanumDrive;

   DriveSpeed driveSpeed = DriveSpeed.Fast;

   boolean slow = false;

   public DriveTrain(HardwareMap hwMap){
       this.mecanumDrive = new PinPoint_MecanumDrive(hwMap,odo);
   }

    @Override
    public void initAuto(HardwareMap hwMap) {
//       mecanumDrive = new SampleMecanumDrive(hwMap);
        odo = hwMap.get(GoBildaPinpointDriver.class,"pinpointodo");
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.optii);
        odo.setOffsets(171.45,0);
//        resetPosAndHeading();
        this.mecanumDrive = new PinPoint_MecanumDrive(hwMap,odo);
//        mecanumDrive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));
    }

    @Override
    public void periodicAuto() {
//        Dashboard.addData("Status", odo.getDeviceStatus());
//        Dashboard.addData("Pinpoint Frequency", odo.getFrequency());
        Dashboard.addData("X_Pos",getXPos());
        Dashboard.addData("Y_Pos",getYPos());
        Dashboard.addData("Heading",getHeading());

        mecanumDrive.update();
        odo.update_LessStuff();
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

    public void setRR_PinPoint(double x, double y, double heading){
        odo.setPosition(new Pose2D(DistanceUnit.INCH,x,y, AngleUnit.DEGREES,heading));
        mecanumDrive.setPoseEstimate(new Pose2d(x,y,heading));
    }

    public void setRR(double x, double y, double heading){
        mecanumDrive.setPoseEstimate(new Pose2d(x,y,heading));
    }

    public void setPinPoint(double x, double y, double heading){
        odo.setPosition(new Pose2D(DistanceUnit.INCH,x,y, AngleUnit.DEGREES,heading));
    }

    public double getXPos(){
        return odo.getPosX();
    }

    public double getYPos(){
        return odo.getPosY();
    }

    public double getHeading(){
        return Math.toDegrees(odo.getHeading());
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

    public void resetPosAndHeading(){
        odo.resetPosAndIMU();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectory){
        mecanumDrive.followTrajectorySequenceAsync(trajectory);
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
    public void lockPosition (Pose2d target){
       Pose2d difference = target.minus(poseEstimate());
       Vector2d xy = difference.vec().rotated(-poseEstimate().getHeading());

       double heading = Angle.normDelta(target.getHeading()) - Angle.normDelta(poseEstimate().getHeading());
       mecanumDrive.setWeightedDrivePower(new Pose2d(xy.times(xyP),heading * headingP));
    }


    public enum DriveSpeed{
       Fast,
        Slow
    }
}
