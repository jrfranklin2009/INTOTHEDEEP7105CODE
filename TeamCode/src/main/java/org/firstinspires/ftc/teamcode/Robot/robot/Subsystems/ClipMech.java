package org.firstinspires.ftc.teamcode.Robot.robot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Subsystem;

@Config
public class ClipMech extends Subsystem {

    Servo rightmagarm,leftmagarm;

    public static double fully_up = .8,ready = .7,outtheway = .5, down = 0;

    AnalogInput clipAnalog;

    @Override
    public void initAuto(HardwareMap hwMap) {
        rightmagarm = hwMap.get(Servo.class,"rightmagarm");
        leftmagarm = hwMap.get(Servo.class,"leftmagarm");
        rightmagarm.setDirection(Servo.Direction.REVERSE);
        clipAnalog = hwMap.get(AnalogInput.class, "clipanalog");
    }

    @Override
    public void periodicAuto() {
        Dashboard.addData("clipmagpos",getClipMagPos());
    }

    public double getClipMagPos(){
        return clipAnalog.getVoltage() / 3.3 * 360;
    }

    public double getVoltage(){
        return clipAnalog.getVoltage();
    }

    @Override
    public void shutdown() {
    }
    public void setArmStates(ArmStates armStates){
        switch (armStates){
            case Clippity_Clappity_Clickity_Click:
                rightmagarm.setPosition(fully_up);
                leftmagarm.setPosition(fully_up);
                break;
            case READY:
                rightmagarm.setPosition(ready);
                leftmagarm.setPosition(ready);
                break;
            case Out_The_Way:
                rightmagarm.setPosition(outtheway);
                leftmagarm.setPosition(outtheway);
                break;
            case Almost_Down:
                rightmagarm.setPosition(.19);
                leftmagarm.setPosition(.19);
                break;
            case Down:
                rightmagarm.setPosition(down);
                leftmagarm.setPosition(down);
                break;
        }
    }

    public enum ArmStates{
        Clippity_Clappity_Clickity_Click, // This states is for fully engaging the clips
        READY,
        Out_The_Way,
        Almost_Down,
        Down
    }
}
