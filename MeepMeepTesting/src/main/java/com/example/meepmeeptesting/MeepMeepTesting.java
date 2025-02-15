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



        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10-72, 18-72, Math.toRadians(70)))
//                        .strafeToLinearHeading(new Vector2d(62,19-72), Math.toRadians(70))
//                        .waitSeconds(0.2)
//                        .strafeToLinearHeading(new Vector2d(59,19-72), Math.toRadians(90))
//                .waitSeconds(0.2)
//                .strafeToLinearHeading(new Vector2d(48,19-72), Math.toRadians(90))
//                .waitSeconds(0.2)
//                .strafeToLinearHeading(new Vector2d(37,8-72), Math.toRadians(90))
                .splineTo(new Vector2d(-57,-31), Math.toRadians(70),new TranslationalVelConstraint(55 ), new ProfileAccelConstraint(-45, 80))

                .splineTo(new Vector2d(48-72,60.5-72), Math.toRadians(0),new TranslationalVelConstraint(55 ), new ProfileAccelConstraint(-45, 80))










                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}