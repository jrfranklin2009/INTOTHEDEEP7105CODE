package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.PinPoint.PinPoint_Odo;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 4096;
    public static double WHEEL_RADIUS = 0.6889764; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = 0; // X is the up and down direction
    public static double PARALLEL_Y = 6.25; // Y is the strafe direction

    public static double PERPENDICULAR_X = -6;
    public static double PERPENDICULAR_Y = 0;

    GoBildaPinpointDriver odo;

    PinPoint_Odo pinPointOdo;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
//    private Encoder parallelEncoder, perpendicularEncoder; // The parallel wheel is the X wheel. The perpendicular wheel measures the Y;

    private PinPoint_MecanumDrive drive;

    public TwoWheelTrackingLocalizer(PinPoint_MecanumDrive drive, GoBildaPinpointDriver odo, DriveTrain drivetrain) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.odo = odo;
        this.drive = drive;
        this.pinPointOdo = pinPointOdo;

//        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
//        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftback"));
//        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightback"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(odo.getEncoderX()),
                encoderTicksToInches(odo.getEncoderY())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(pinPointOdo.getCorrectedVelocity(odo.getVelX())),
                encoderTicksToInches(pinPointOdo.getCorrectedVelocity(odo.getVelY()))
        );
    }
}