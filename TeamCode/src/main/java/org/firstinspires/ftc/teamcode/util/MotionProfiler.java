package org.firstinspires.ftc.teamcode.util;//package org.firstinspires.ftc.teamcode.RR_Quickstart.util;
//
//import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
//import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
//import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
//import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
//import com.acmerobotics.roadrunner.profile.MotionProfile;
//import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
//import com.acmerobotics.roadrunner.profile.MotionState;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//
//public class MotionProfiler {
//
//    MotionProfile motionProfile;
//
//    ElapsedTime timer = new ElapsedTime();
//
//    BasicPID pid;
//
//    PIDCoefficients coefficients;
//
//    double MaxVel,MaxAccel,MaxNegAccel;
//
//    double motor_targetPos = 0,motor_state = 0, prev_MotorTarget = 0;
//
//    public MotionProfiler(double MaxVel, double MaxAccel, double MaxNegAccel, PIDCoefficients coefficients){
//        this.MaxVel = MaxVel;
//        this.MaxAccel = MaxAccel;
//        this.MaxNegAccel = MaxNegAccel;
//        this.coefficients = coefficients;
//        this.pid = new BasicPID(coefficients);
//    }
//
//    public double calculate(double target, double state){
//        motor_targetPos = getTargetPos();
//        motor_state = state;
//        newProfile(target,state);
//        return pid.calculate(target,state);
//    }
//
//    public double getTargetPos(){
//        return motionProfile.get(timer.seconds()).getX();
//    }
//
//    public void newProfile(double target, double state){
//        motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
//                new MotionState(state, 0, 0),
//                new MotionState(target, 0, 0),
//                MaxVel,
//                MaxAccel
//        );
//    }
//}
