package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

/** Testing for meepmeep.  Plan is to use this to set out coordinates for pathing robot.
 * We can fine tune on the actual field, but we want to get the general coordinates laid out using meep meep.
 */

public class MeepMeepTesting {


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\swift\\OneDrive\\Documents\\field.png")); }
        catch (IOException e) {
            e.printStackTrace();
        }

        meepMeep.setBackground(img);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)  // set parameters for meep meep
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -61, Math.toRadians(90)))  // set bot's initial position
                                .forward(24)
                                .waitSeconds(0.3)
                                .forward(24)
                                .strafeRight(48)
                                .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(0)))  // go and place
                                .lineToLinearHeading(new Pose2d(48, -60, Math.toRadians(0)))  // park
                                .forward(12)
                                .build()
                );

        meepMeep.addEntity(myBot).start();

    }
}
