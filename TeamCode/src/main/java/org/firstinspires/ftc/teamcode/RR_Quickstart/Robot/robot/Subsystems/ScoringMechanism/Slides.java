package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.ScoringMechanism;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.SlideTurret.pivot1;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Controllers.MotionProfileConstraint;
import org.firstinspires.ftc.teamcode.Controllers.MotionProfiler;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.SimpleCommands.RunKinematics;
import org.firstinspires.ftc.teamcode.Robot.Input;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;

@Config
public class Slides extends Subsystem {
    public static double HighPos = 1450, MidPos = 550, MaxAccel = 30, MaxNegAccel = 65, MaxVel = 60;
    public static double kp = 0.008, ki = 0, kd = 0.00004, kg = .18, ref = 0;
    public static TouchSensor touchslides;
    public static double counter = 1;
    PIDCoefficients coefficients = new PIDCoefficients(kp,ki,kd);
    BasicPID pid = new BasicPID(coefficients);
    MotionProfileConstraint upwardsconstraint = new MotionProfileConstraint(MaxAccel,MaxAccel,MaxVel);
    MotionProfileConstraint downconstraint = new MotionProfileConstraint(MaxAccel, MaxAccel,MaxVel);
    MotionProfiler motionProfiler = new MotionProfiler(upwardsconstraint,downconstraint, coefficients);
    public DcMotorEx leftslide, rightslide;
    @Override
    public void initAuto(HardwareMap hwMap) {
        leftslide = hwMap.get(DcMotorEx.class,"leftslide");
        rightslide = hwMap.get(DcMotorEx.class,"rightslide");
        touchslides = hwMap.get(TouchSensor.class,"touchslides");
        leftslide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ref = 0;
        counter = 1;
    }

    @Override
    public void periodicAuto() {
//        Dashboard.packet.put("slideticks",getSlidePos());  // get position of slides
//        Dashboard.packet.put("targetpos",ref);
//        Dashboard.packet.put("SlidePos", getSlidePos());
    }

    @Override
    public void shutdown() {
        leftslide.setPower(0);
        rightslide.setPower(0);
    }

    public void calculate(Input input, Input input2) {
        if (touchslides.isPressed() && getSlidePos() < 10 && !input2.isLeftBumperPressed() && !input2.isRightBumperPressed() &&
                !input.getGamepad().dpad_up && !input.getGamepad().dpad_down) {
            leftslide.setPower(0);
            rightslide.setPower(0);
        } else if (pivot1.getPosition() == 0) {
            leftslide.setPower(0);
            rightslide.setPower(0);
        } else if (!touchslides.isPressed() && getSlidePos() < 50) {
            leftslide.setPower(-.4);
            rightslide.setPower(-.4);
        } else if (input2.isRightBumperPressed() && getSlidePos() < 1400) {
            ref = ref + 200;
            leftslide.setPower(pid.calculate(ref, getSlidePos()));
            rightslide.setPower(pid.calculate(ref, getSlidePos()));
        }  else if (input2.isLeftBumperPressed() && getSlidePos() > 50) {
            ref = ref - 200;
            leftslide.setPower(pid.calculate(ref, getSlidePos()));
            rightslide.setPower(pid.calculate(ref, getSlidePos()));
        } else if (input2.getLeft_stick_y() > .9 && getSlidePos() > 50) {
            ref = 0;
            leftslide.setPower(pid.calculate(ref, getSlidePos()));
            rightslide.setPower(pid.calculate(ref, getSlidePos()));
        }  else if (input.getGamepad().dpad_up) {
            ref = HighPos;
            leftslide.setPower(pid.calculate(ref, getSlidePos()));
            rightslide.setPower(pid.calculate(ref, getSlidePos()));
        } else if (input.getGamepad().dpad_down) {
            ref = MidPos;
            leftslide.setPower(pid.calculate(ref, getSlidePos()));
            rightslide.setPower(pid.calculate(ref, getSlidePos()));
        } else {
            leftslide.setPower(pid.calculate(ref, getSlidePos()));
            rightslide.setPower(pid.calculate(ref, getSlidePos()));
        }
    }

    public void calculatePreSet2Kinematics(Input input, Input input2, double rat, Intake intake, RunKinematics kinematics) {
        if (input2.isRightBumperPressed() && ref <= 1540) {
            ref = ref + 220;
        }
        // go down
        else if (input2.isLeftBumperPressed() && ref >= 220) {
            ref = ref - 220;
        }
        // going down easy for brady
        else if (input2.getLeft_stick_y() > .9) {
            ref = 0;
            leftslide.setPower(pid.calculate(ref, getSlidePos()));
            rightslide.setPower(pid.calculate(ref, getSlidePos()));
        }  else if (input.getGamepad().dpad_up) {
            ref = HighPos;
        } else if (input.getGamepad().dpad_down) {
            ref = MidPos;
            intake.setIntakeHeight(Intake.IntakeHeightStates.Hang);
        } else {
            if (rat < -1000 && !input2.isLeftTriggerPressed() && !input2.isRightTriggerPressed()) {
                kinematics.periodic();
                leftslide.setPower(pid.calculate(ref, getSlidePos()));
                rightslide.setPower(pid.calculate(ref, getSlidePos()));
            }
        }
    }

    public void calculatePreSet2(Input input, Input input2, double rat, Intake intake) {
            if (input2.isRightBumperPressed() && ref <= 1540) {
            ref = ref + 220;
        }
        // go down
        else if (input2.isLeftBumperPressed() && ref >= 220) {
            ref = ref - 220;
        }
        // going down easy for brady
        else if (input2.getLeft_stick_y() > .9) {
            ref = 0;
            leftslide.setPower(pid.calculate(ref, getSlidePos()));
            rightslide.setPower(pid.calculate(ref, getSlidePos()));
        }  else if (input.getGamepad().dpad_up) {
            ref = HighPos;
        } else if (input.getGamepad().dpad_down) {
            ref = MidPos;
            intake.setIntakeHeight(Intake.IntakeHeightStates.Hang);
        } else {
             if (rat < -1000 && !input2.isLeftTriggerPressed() && !input2.isRightTriggerPressed()) {
                 leftslide.setPower(pid.calculate(ref, getSlidePos()));
                 rightslide.setPower(pid.calculate(ref, getSlidePos()));
             }
         }
    }




    public void calculatePreSet(Input input, Input input2) {
            if (pivot1.getPosition() == 0) {
//            counter = 1;
            leftslide.setPower(0);
            rightslide.setPower(0);
        }
        // go up
        else if (input2.isRightBumperPressed() && getSlidePos() < 1400 && ref == 0) {
            counter = 0;
            ref = ref + 220;
        } else if (input2.isRightBumperPressed() && getSlidePos() < 1400 && ref == 220) {
            counter = 0;
            ref = ref + 220;
        } else if (input2.isRightBumperPressed() && getSlidePos() < 1400 && ref == 440) {
            counter = 0;
            ref = ref + 220;
        } else if (input2.isRightBumperPressed() && getSlidePos() < 1400 && ref == 660) {
            counter = 0;
            ref = ref + 220;
        } else if (input2.isRightBumperPressed() && getSlidePos() < 1400 && ref == 880) {
            counter = 0;
            ref = ref + 220;
        } else if (input2.isRightBumperPressed() && getSlidePos() < 1400 && ref == 1100) {
                counter = 0;
                ref = ref + 220;
            }

        // go down
        else if (input2.isLeftBumperPressed() && getSlidePos() > 50 && ref == 1100) {
                counter = 0;
                ref = ref - 220;
        } else if (input2.isLeftBumperPressed() && getSlidePos() > 50 && ref == 880) {
            counter = 0;
            ref = ref - 220;
        } else if (input2.isLeftBumperPressed() && getSlidePos() > 50 && ref == 660) {
            counter = 0;
            ref = ref - 220;
        } else if (input2.isLeftBumperPressed() && getSlidePos() > 50 && ref == 440) {
            counter = 0;
            ref = ref - 220;
        } else if (input2.isLeftBumperPressed() && getSlidePos() > 50 && ref == 220) {
            counter = 0;
            ref = ref - 220;
        }
        // going down easy for brady
        else if (input2.getLeft_stick_y() > .9 && getSlidePos() > 50) {
            ref = 0;
            counter = 1;
            leftslide.setPower(pid.calculate(ref, getSlidePos()));
            rightslide.setPower(pid.calculate(ref, getSlidePos()));
        }  else if (input.getGamepad().dpad_up) {
            ref = HighPos;
        } else if (input.getGamepad().dpad_down) {
            ref = MidPos;
        } else if (counter == 2) {
            leftslide.setPower(pid.calculate(ref, getSlidePos()));
            rightslide.setPower(pid.calculate(ref, getSlidePos()));
        } else if (input2.getLeft_stick_y() < -.9 && counter == 0) {
            counter = 2;
        }
    }

    public void calculatMotionProfile(Input input, Input input2){
        double power = motionProfiler.calculate(400,getSlidePos());
        double power2 = motionProfiler.calculate(0,getSlidePos());
        if (input2.isRightBumperPressed()){
            rightslide.setPower(power);
            leftslide.setPower(power);
        } else if (input2.isLeftBumperPressed()) {
            rightslide.setPower(power2);
            leftslide.setPower(power2);
        }
    }
//    public void calculatMotionProfile(Input input, Input input2) {
//        if (touchslides.isPressed() && getSlidePos() < 10 && !input2.isLeftBumperPressed() && !input2.isRightBumperPressed() &&
//                !input.getGamepad().dpad_up && !input.getGamepad().dpad_down) {
//            leftslide.setPower(0);
//            rightslide.setPower(0);
//        } else if (pivot1.getPosition() == 0) {
//            leftslide.setPower(0);
//            rightslide.setPower(0);
//        } else if (!touchslides.isPressed() && getSlidePos() < 10) {
//            leftslide.setPower(-.4);
//            rightslide.setPower(-.4);
//        } else if (input2.isRightBumperPressed() && getSlidePos() < 1400) {
//            ref = ref + 200;
//            leftslide.setPower(motionProfiler.calculate(ref, getSlidePos()));
//            rightslide.setPower(motionProfiler.calculate(ref, getSlidePos()));
//        }  else if (input2.isLeftBumperPressed() && getSlidePos() > 50) {
//            ref = ref -200;
//            leftslide.setPower(motionProfiler.calculate(ref, getSlidePos()));
//            rightslide.setPower(motionProfiler.calculate(ref, getSlidePos()));
//        } else if (input2.getLeft_stick_y() > .9 && getSlidePos() > 50) {
//            ref = 0;
//            leftslide.setPower(motionProfiler.calculate(ref, getSlidePos()));
//            rightslide.setPower(motionProfiler.calculate(ref, getSlidePos()));
//        }  else if (input.getGamepad().dpad_up) {
//            ref = HighPos;
//            leftslide.setPower(motionProfiler.calculate(ref, getSlidePos()));
//            rightslide.setPower(motionProfiler.calculate(ref, getSlidePos()));
//        } else if (input.getGamepad().dpad_down) {
//            ref = MidPos;
//            leftslide.setPower(motionProfiler.calculate(ref, getSlidePos()));
//            rightslide.setPower(motionProfiler.calculate(ref, getSlidePos()));
//        } else {
//            leftslide.setPower(motionProfiler.calculate(ref, getSlidePos()));
//            rightslide.setPower(motionProfiler.calculate(ref, getSlidePos()));
//        }
//    }

    public double setTarget(double target){
        return ref = target;
    }

    public double getTarget(){
        return ref;
    }

    public void resetSlides() {
        leftslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void calculateAuto() {
            leftslide.setPower(pid.calculate(ref, getLeftSlidePos()));
            rightslide.setPower(pid.calculate(ref, getLeftSlidePos()));
    }

    public double getSlidePos(){
        return (leftslide.getCurrentPosition() + rightslide.getCurrentPosition()) / 2;
    }

    public double getLeftSlidePos(){
        return leftslide.getCurrentPosition();
    }

    public double getError(double target){
        return target - getSlidePos();
    }
}
