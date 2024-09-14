package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands;

import android.transition.Slide;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.RR_Quickstart.CommandFrameWork.Command;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Input;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.ScoringMechanism.Slides;

public class ScoringCommandGroups {

    Slides slides;
    Intake intake;

    public ScoringCommandGroups(Slides slides, Intake intake) {
        this.slides = slides;
        this.intake = intake;
    }


}