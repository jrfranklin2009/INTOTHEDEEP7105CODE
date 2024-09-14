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
      //  mecanumDrive.setZeroPowerBehaviorAllMotors();
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
//           if ((colorSensor1.getNormalizedColors().red > 0.3 & colorSensor2.getNormalizedColors().red > 0.3) || (colorSensor1.getNormalizedColors().blue > 0.3 & colorSensor2.getNormalizedColors().blue > 0.3))  {
           mecanumDrive.setWeightedDrivePower(new Pose2d(
                   (-input.getLeft_stick_y() * .3),
                   (-input.getLeft_stick_x() * .3),
                   (-input.getRight_stick_x() * .3)));
           }
       }
//    }

//<<<<<<< HEAD
//=======
//    public void RobotRelativeExperimental(Input input){
//        if (driveSpeed == DriveSpeed.Fast) {
//            mecanumDrive.setWeightedDrivePower(new Pose2d(
//                    -input.getLeft_stick_y(),
//                    -input.getLeft_stick_x(),
//                    -input.getRight_stick_x()));
//        } else if (driveSpeed == DriveSpeed.Slow) {
////            if ((colorSensor1.getNormalizedColors().red > 0.3 & colorSensor2.getNormalizedColors().red > 0.3) || (colorSensor1.getNormalizedColors().blue > 0.3 & colorSensor2.getNormalizedColors().blue > 0.3)) {
//                mecanumDrive.setWeightedDrivePower(new Pose2d(
//                        Math.abs(-input.getLeft_stick_y() * .3),
//                        -input.getLeft_stick_x() * .3,
//                        -input.getRight_stick_x() * .3));
//            }
//            else if ((colorSensor1.getNormalizedColors().red > 0.3) || (colorSensor1.getNormalizedColors().blue > 0.3))  {
//                Pose2d positionPower = new Pose2d(Math.abs(-input.getLeft_stick_y()*0.3), -input.getLeft_stick_x()*0.3, -input.getRight_stick_x()*0.3);
//                if (Math.abs(positionPower.getX())+Math.abs(positionPower.getY())+Math.abs(positionPower.getHeading())>1) {
//                    positionPower = new Pose2d(mecanumDrive.VX_WEIGHT*positionPower.getX(), mecanumDrive.VY_WEIGHT*positionPower.getY(), mecanumDrive.OMEGA_WEIGHT*positionPower.getHeading()).div(mecanumDrive.VX_WEIGHT*Math.abs(positionPower.getX())+mecanumDrive.VY_WEIGHT*Math.abs(positionPower.getY())+mecanumDrive.OMEGA_WEIGHT*Math.abs(positionPower.getHeading()));
//                }
//
//                List MotorPowers = MecanumKinematics.robotToWheelVelocities(positionPower, 1.0, 1.0, SampleMecanumDrive.LATERAL_MULTIPLIER);
//
//                mecanumDrive.leftFront.setPower(0);
//                mecanumDrive.leftRear.setPower(0);
//                mecanumDrive.rightRear.setPower((Double) MotorPowers.get(3));
//                mecanumDrive.rightFront.setPower((Double) MotorPowers.get(4));
//            }
//            else if ((colorSensor2.getNormalizedColors().red > 0.3) || (colorSensor2.getNormalizedColors().blue > 0.3))  {
//                Pose2d positionPower = new Pose2d(Math.abs(-input.getLeft_stick_y()*0.3), -input.getLeft_stick_x()*0.3, -input.getRight_stick_x()*0.3);
//                if (Math.abs(positionPower.getX())+Math.abs(positionPower.getY())+Math.abs(positionPower.getHeading())>1) {
//                    positionPower = new Pose2d(mecanumDrive.VX_WEIGHT*positionPower.getX(), mecanumDrive.VY_WEIGHT*positionPower.getY(), mecanumDrive.OMEGA_WEIGHT*positionPower.getHeading()).div(mecanumDrive.VX_WEIGHT*Math.abs(positionPower.getX())+mecanumDrive.VY_WEIGHT*Math.abs(positionPower.getY())+mecanumDrive.OMEGA_WEIGHT*Math.abs(positionPower.getHeading()));
//                }
//
//                List MotorPowers = MecanumKinematics.robotToWheelVelocities(positionPower, 1.0, 1.0, SampleMecanumDrive.LATERAL_MULTIPLIER);
//
//                mecanumDrive.leftFront.setPower((Double) MotorPowers.get(2));
//                mecanumDrive.leftRear.setPower((Double) MotorPowers.get(1));
//                mecanumDrive.rightRear.setPower(0);
//                mecanumDrive.rightFront.setPower(0);
//            }
//
//        }
//    }

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
