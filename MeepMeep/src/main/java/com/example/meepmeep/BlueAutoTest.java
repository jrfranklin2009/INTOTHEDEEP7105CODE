package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

/** Testing for meepmeep.  Plan is to use this to set out coordinates for pathing robot.
 * We can fine tune on the actual field, but we want to get the general coordinates laid out using meep meep.
 */

public class BlueAutoTest {


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);





        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 10, Math.toRadians(90), Math.toRadians(90), 16.5)  // set parameters for meep meep
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-15.8, -59.7, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-15.8, -46, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-48,-46, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-52.7, -52.7, Math.toRadians(45) ))
                                .lineToLinearHeading(new Pose2d(-59.9, -46, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-52.7, -52.7, Math.toRadians(45) ))
                                .lineToLinearHeading(new Pose2d(-48, -46, Math.toRadians(135)))
                                .lineToLinearHeading(new Pose2d(-52.7, -52.7, Math.toRadians(45) ))
                        .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL);

        meepMeep.addEntity(myBot).start();

    }
}
