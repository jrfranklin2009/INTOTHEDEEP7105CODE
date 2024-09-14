package org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Commands.ScoringCommands;

import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.RR_Quickstart.Robot.robot.Subsystems.DepositingMechanisms.Slides;

public class ScoringCommandGroups {

    Slides slides;
    Intake intake;

    public ScoringCommandGroups(Slides slides, Intake intake) {
        this.slides = slides;
        this.intake = intake;
    }


}