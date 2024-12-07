package org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.robot.Subsystems.Dashboard;

@Config
public class JohnsIntake extends Subsystem {

    CRServo rightintake,leftintake;
    Servo gripper,rightarm,leftarm;

    public static double outback = .78, down = 0.165;

    NormalizedColorSensor colorsensor;

    AnalogInput armanalog;

    @Override
    public void initAuto(HardwareMap hwMap) {
        rightintake = hwMap.get(CRServo.class,"rightintake");
        leftintake = hwMap.get(CRServo.class,"leftintake");
        gripper = hwMap.get(Servo.class,"gripper");
        rightarm = hwMap.get(Servo.class,"rightarm");
        leftarm = hwMap.get(Servo.class,"leftarm");
        armanalog = hwMap.get(AnalogInput.class,"armanalog");

        leftintake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightarm.setDirection(Servo.Direction.REVERSE);

        colorsensor = hwMap.get(NormalizedColorSensor.class,"colorsensor");
    }

    @Override
    public void periodicAuto() {
        Dashboard.addData("blue",getBlue());
        Dashboard.addData("red",getRed());
        Dashboard.addData("green",getGreen());
    }

    @Override
    public void shutdown() {

    }

    public double getArmPos(){
        return armanalog.getVoltage() / 3.3 * 360;
    }

    public double getBlue(){
        return colorsensor.getNormalizedColors().blue;
    }

    public double getRed(){
        return colorsensor.getNormalizedColors().red;
    }

    public double getGreen(){
        return colorsensor.getNormalizedColors().green;
    }

    public void getColor(SampleStates samplestates){
        switch (samplestates){
            case RED:

                break;
            case BLUE:

                break;
            case YELLOW:

                break;
            case READ:
//                if (colorsensor instanceof SwitchableLight) {
                    ((SwitchableLight)colorsensor).enableLight(true);
//                }
                if (getRed() > 200){
                    samplestates = SampleStates.RED;
                } else if (getBlue() > 200){
                    samplestates = SampleStates.BLUE;
                } else if (getBlue() > 150 && getRed() > 150) {
                    samplestates = SampleStates.YELLOW;
                }
                break;
            case SHUT_OFF:
                ((SwitchableLight)colorsensor).enableLight(false);
                break;
        }
    }

    public void setIntake(IntakeStates intakeStates){
        switch (intakeStates){
            case intake:
                rightintake.setPower(1);
                leftintake.setPower(1);
                break;
            case outtake:
                rightintake.setPower(-.8);
                leftintake.setPower(-.8);
                break;
            case stop:
                rightintake.setPower(0);
                leftintake.setPower(0);
                break;
        }
    }

    public void setGripper(GripperStates gripperStates) {
        switch (gripperStates){
            case unclamp:
                gripper.setPosition(.1);
                break;
            case clamp:
                gripper.setPosition(.77);
                break;
        }
    }

    public void setArmStates(ArmStates armStates){
        switch (armStates){
            case forward:
                rightarm.setPosition(down); // 121
                leftarm.setPosition(down);


                break;
            case parallel:
                rightarm.setPosition(.26);
                leftarm.setPosition(.26);
                break;
            case snapclip:
                rightarm.setPosition(.23);
                leftarm.setPosition(.23); //143
                break;
            case outback:
                rightarm.setPosition(outback);//227
                leftarm.setPosition(outback);
                break;
        }
    }

    public enum SampleStates{
        BLUE,
        RED,
        YELLOW,
        READ,
        SHUT_OFF
    }

    public enum IntakeStates{
        intake,
        outtake,
        stop
    }
    public enum GripperStates{
        clamp,
        unclamp
    }
    public enum ArmStates{
        outback,
        snapclip,
        parallel,
        forward
    }
}
