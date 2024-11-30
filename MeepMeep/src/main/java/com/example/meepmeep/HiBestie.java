package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

/** Testing for meepmeep.  Plan is to use this to set out coordinates for pathing robot.
 * We can fine tune on the actual field, but we want to get the general coordinates laid out using meep meep.
 */

public class HiBestie {


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);





        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 10, Math.toRadians(90), Math.toRadians(90), 16)  // set parameters for meep meep
                .followTrajectorySequence(drive ->
                       drive.trajectorySequenceBuilder(new Pose2d(-9, 61, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-9, 34, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-9,47, Math.toRadians(0)))
                        //.lineToLinearHeading(new Pose2d(-45, -30, Math.toRadians(155)))
                        //.lineToLinearHeading(new Pose2d(-58,-63.5, Math.toRadians(270)))
                        //.lineToLinearHeading(new Pose2d(-55,-50, Math.toRadians(270)))
                        //.lineToLinearHeading(new Pose2d(-56.5,-49, Math.toRadians(90)))
                       // .lineToLinearHeading(new Pose2d(-56.5,-30, Math.toRadians(90)))
                       // .lineToLinearHeading(new Pose2d(-56.5,-31, Math.toRadians(270)))
                      //  .lineToLinearHeading(new Pose2d(-58,-63.5, Math.toRadians(270)))
                        //.lineToLinearHeading(new Pose2d(-59,-61, Math.toRadians(270)))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL);

        meepMeep.addEntity(myBot).start();

    }
}
