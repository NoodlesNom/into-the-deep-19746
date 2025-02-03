package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 80, 6, 5, 11.75)
                .build();



        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(62.5, 23-72, Math.toRadians(90)))
                        .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(46,23-72, Math.toRadians(90)), Math.toRadians(180),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-30, 70))
                .splineToLinearHeading(new Pose2d(36,15-72, Math.toRadians(90)), Math.toRadians(270),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-30, 70))

                .splineToLinearHeading(new Pose2d(36,7.25-72, Math.toRadians(90)), Math.toRadians(270),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-30, 70))









                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}