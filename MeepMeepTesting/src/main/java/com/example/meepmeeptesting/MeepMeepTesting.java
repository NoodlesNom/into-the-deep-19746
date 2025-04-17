package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {
        int angle = 53;
        int blocky =-2;
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 80, 6, 5, 11.75)
                .build();
//.splineToSplineHeading(new Pose2d(24-72,40+blocky*0.7-72, Math.toRadians(62+blocky/2)), Math.toRadians(62+blocky/2),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
//
//                .splineToSplineHeading(new Pose2d(46-72,60.5+blocky-72, Math.toRadians(0)), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
//                //.splineToSplineHeading(new Pose2d(47-72,60.5-72, Math.toRadians(0)), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
//
//                .splineToSplineHeading(new Pose2d(49-72,60.5+blocky-72, Math.toRadians(0)), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))



        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(49-72, 60.5-72+5, Math.toRadians(0)))
                        .setReversed(true)
                .splineTo(new Vector2d(46-72,60.5-72+(5)), Math.toRadians(0-180))

                .splineTo(new Vector2d(20-72,40-72+(5)*0.2), Math.toRadians(62+(5)*(0.5)-180))
                .splineTo(new Vector2d(13-72,23-72), Math.toRadians(70-180))

                .build());
//        .setTangent(Math.toRadians(angle))
//                .splineToSplineHeading(new Pose2d(16+Math.sin(Math.toRadians(angle-90))*10, 38-10*Math.cos(Math.toRadians(angle-90))-72, Math.toRadians(angle)), Math.toRadians(angle),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
//
//                .splineToSplineHeading(new Pose2d(16+Math.sin(Math.toRadians(angle-90))*5, 38-5*Math.cos(Math.toRadians(angle-90))-72, Math.toRadians(angle)), Math.toRadians(angle),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))





        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}