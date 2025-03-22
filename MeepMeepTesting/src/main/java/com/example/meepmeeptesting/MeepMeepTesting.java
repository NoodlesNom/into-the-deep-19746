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
        int angle = 123;
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



        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(16, 7, Math.toRadians(0)))
                //.splineTo(new Vector2d(48-72,60.5-72), Math.toRadians(0),new TranslationalVelConstraint(55 ), new ProfileAccelConstraint(-45, 80))
                //.splineToLinearHeading(new Pose2d(44-72,60.5-72, Math.toRadians(0)), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))z
                //.setTangent(Math.toRadians(70))
                //.setReversed(true)
                //.setTangent(Math.toRadians(100-180))
                //.splineToLinearHeading(new Pose2d(8.5, 32-72 ,Math.toRadians(90)), Math.toRadians(315),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                                //.setReversed(true)
                //.splineToLinearHeading(new Pose2d(33, 20-72 ,Math.toRadians(90)), Math.toRadians(145-180),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 85))
                //.splineToLinearHeading(new Pose2d(39, 12-72 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                //.splineTo(new Vector2d( 36, 8-72), Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                        .setReversed(true)
                .splineTo(new Vector2d(-40, 8), Math.toRadians(180),new TranslationalVelConstraint(70 ), new ProfileAccelConstraint(-45, 95))
                .splineTo(new Vector2d(-55, 6), Math.toRadians(210),new TranslationalVelConstraint(70 ), new ProfileAccelConstraint(-45, 95))





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