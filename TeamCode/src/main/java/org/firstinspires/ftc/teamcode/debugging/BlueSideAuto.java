package org.firstinspires.ftc.teamcode.debugging;

import static org.firstinspires.ftc.teamcode.autonomous.gf.GFMovement.followCurve;

import org.firstinspires.ftc.teamcode.autonomous.gf.CurvePoint;
import org.firstinspires.ftc.teamcode.autonomous.gf.GFMovement;

import java.util.ArrayList;

public class BlueSideAuto extends AutoTesting {
    public enum progStates {
        splineToPickup
    }

    private double SPEED_SCALE = 2.1;

    private final double stack1X = 60;
    private final double stack1Y = 130.0;

    private final double stack2X = 54;
    private final double stack2Y = 130.5;

    private final double backboardX = 56;
    private final double backboardY = 19.5;

    private final double backstageX= 60;
    private final double backstageY = 19.5;
    private boolean done = false;

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void MainStateMachine() {
        super.MainStateMachine();

        if (programStage == progStates.splineToPickup.ordinal()) {
            if (stageFinished) {
                System.out.println("Test");
                RobotTesting.setPosition(
                        0, 0, Math.toRadians(90));
                initializeStateVariables();
            }

            double vScale = 1.0;

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(0,0,
                    0, 0, 0, 0, 0, 0));

            // x was 15.8
            points.add(new CurvePoint(24, 0,
                    1, 1,40,40,
                    Math.toRadians(60),0.6));
            points.add(new CurvePoint(32, -1,
                    1, 1,40,40,
                    Math.toRadians(60),0.6));
            points.add(new CurvePoint(32.2, -2,
                    1, 1,40,40,
                    Math.toRadians(60),0.6));



            GFMovement.updateRobotVars(getXPos(), getYPos(), getAngle_rad(), SpeedOmeter.getSpeedX(), SpeedOmeter.getSpeedY(), SpeedOmeter.getRadPerSecond());

            if (done){
                RobotTesting.xSpeed = 0;
                RobotTesting.ySpeed = 0;
                RobotTesting.turnSpeed = 0;
            }else{
                done=followCurve(points, Math.toRadians(180),false);

            }



        }
    }
}
