package org.firstinspires.ftc.teamcode.Robot.robot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Subsystem;

@Config
public class ClipMech extends Subsystem {

    Servo rightmagarm,leftmagarm;

    public static double fully_up = 0.385,ready = .3,outtheway = .2, down = 0;

    @Override
    public void initAuto(HardwareMap hwMap) {
        rightmagarm = hwMap.get(Servo.class,"rightmagarm");
        leftmagarm = hwMap.get(Servo.class,"leftmagarm");
        rightmagarm.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void periodicAuto() {

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
                rightmagarm.setPosition(.094);
                leftmagarm.setPosition(.094);
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
