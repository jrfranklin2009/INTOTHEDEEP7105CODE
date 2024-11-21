package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

/** Testing for meepmeep.  Plan is to use this to set out coordinates for pathing robot.
 * We can fine tune on the actual field, but we want to get the general coordinates laid out using meep meep.
 */

public class Blue3WeekMiddleAuto {


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);





        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)  // set parameters for meep meep
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-63, 0, Math.toRadians(0)))  // set bot's initial position
                                .lineToLinearHeading(new Pose2d(-32, 0, Math.toRadians(0)))
                                // clip specimen
                                .lineToLinearHeading(new Pose2d(-50,0, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-23, 40, Math.toRadians(90)))
                                // pick up first yellow sample
                                .lineToLinearHeading(new Pose2d(-53, 53, Math.toRadians(135)))
                                // deposit sample
                                .lineToLinearHeading(new Pose2d(-23, 49, Math.toRadians(90)))
                                // pick up second yellow sample
                                .lineToLinearHeading(new Pose2d(-53, 53, Math.toRadians(135)))
                                // deposit second yellow sample
                                .lineToLinearHeading(new Pose2d(-23, 58, Math.toRadians(90)))
                                // pick up third yellow sample
                                .lineToLinearHeading(new Pose2d(-53, 53, Math.toRadians(135)))
                                // deposit third sample
                                .lineToLinearHeading(new Pose2d(-50, 0, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-32, 0, Math.toRadians(0)))
                                // park
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK);

        meepMeep.addEntity(myBot).start();

    }
}
