package org.firstinspires.ftc.teamcode.Robot.robot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandFrameWork.Subsystem;

@Config
public class ClipMech extends Subsystem {

    Servo rightmagarm,leftmagarm;

    public static double fully_up = .4,ready = .3, down = .1;

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
            case Fully_Up:
                rightmagarm.setPosition(fully_up);
                leftmagarm.setPosition(fully_up);
                break;
            case READY:
                rightmagarm.setPosition(ready);
                leftmagarm.setPosition(ready);
                break;
            case Down:
                rightmagarm.setPosition(down);
                leftmagarm.setPosition(down);
                break;
        }
    }

    public enum ArmStates{
        Fully_Up,
        READY,
        Down
    }
}
