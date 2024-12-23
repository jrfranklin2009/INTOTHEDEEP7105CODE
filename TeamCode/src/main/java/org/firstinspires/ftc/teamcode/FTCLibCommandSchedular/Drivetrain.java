package org.firstinspires.ftc.teamcode.FTCLibCommandSchedular;

import androidx.annotation.GuardedBy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Cyliis.Robot.robot.Input;
import org.firstinspires.ftc.teamcode.Cyliis.Robot.robot.Subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.drive.PinPoint_MecanumDrive;

public class Drivetrain extends BetterSubsystems {
    public PinPoint_MecanumDrive mecanumDrive;

    public static double headingP = 1,xyP = 1,imuAngle, imuOffSet = 0, imuVelocity = 0;

    DriveTrain.DriveSpeed driveSpeed = DriveTrain.DriveSpeed.Fast;
    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public IMU imu;
    private Thread imuThread;

    public static boolean usingThread = true, slow = false;

    public Drivetrain(HardwareMap hwMap){
        imu = hwMap.get(IMU.class,"imu");
        this.mecanumDrive = new PinPoint_MecanumDrive(hwMap);
    }

    @Override
    public void init() {

    }

    @Override
    public void periodic() {

    }

    @Override
    public void shutdown() {

    }

    public void RobotRelative(Input input) {
        if (driveSpeed == DriveTrain.DriveSpeed.Fast) {
            mecanumDrive.setWeightedDrivePower(new Pose2d(
                    -input.getLeft_stick_y(),
                    input.getLeft_stick_x(),
                    -input.getRight_stick_x()));
        } else if (driveSpeed == DriveTrain.DriveSpeed.Slow) {
            mecanumDrive.setWeightedDrivePower(new Pose2d(
                    (-input.getLeft_stick_y() * .3),
                    (input.getLeft_stick_x() * .3),
                    (-input.getRight_stick_x() * .3)));
        }
    }
    public enum DriveSpeed{
        Fast,
        Slow
    }
}

    //    public PinPoint_MecanumDrive mecanumDrive;
//    public GoBildaPinpointDriver odo;
////    public DcMotorEx leftFront,leftRear,rightRear,rightFront;
//    public Drivetrain(final HardwareMap hMap) {
//        odo = hMap.get(GoBildaPinpointDriver.class,"pinpointodo");
//        mecanumDrive.leftFront = hMap.get(DcMotorEx.class, "lf");
//        mecanumDrive.leftRear = hMap.get(DcMotorEx.class, "lb");
//        mecanumDrive.rightRear = hMap.get(DcMotorEx.class, "rb");
//        mecanumDrive.rightFront = hMap.get(DcMotorEx.class, "rf");
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.optii);
//        odo.setOffsets(171.45,0);
//         mecanumDrive = new PinPoint_MecanumDrive(hMap, odo,1,1);
//    }
//
//    @Override
//    public void periodic() {
//        // This method will be called once per scheduler run
//    }

