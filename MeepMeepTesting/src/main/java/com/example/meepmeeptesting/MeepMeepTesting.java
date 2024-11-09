package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d((14.9/2), 25+(17.3/2), Math.toRadians(-90)))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-24,36), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-36,26), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-36,20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-48,13), Math.toRadians(180))
                .waitSeconds(0.1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-58,55, Math.toRadians(-55)), Math.toRadians(-55+180))
                        .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-36,70-(17.3/2)), Math.toRadians(90))
                .waitSeconds(0.2)
                .waitSeconds(0.2)
                .waitSeconds(0.3)
                .strafeToLinearHeading(new Vector2d((14.9/2-5), 25+(17.3/2)), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d((-45), 60), Math.toRadians(-90))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}