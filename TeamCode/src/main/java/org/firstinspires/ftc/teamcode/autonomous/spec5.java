package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.WeakReferenceSet;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.autonomous.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.rr.localizer.PinpointDrive;
import org.firstinspires.ftc.teamcode.util.BotLog;

import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name = "5 spec 🤑", group = "Autonomous")


public class spec5 extends LinearOpMode {
    public Robot robot;
    public ElapsedTime timer;
    ElapsedTime WallTimer = new ElapsedTime();
    boolean debug = true;
    ElapsedTime loopTimer = new ElapsedTime();
    Deadline logging = new Deadline(200, TimeUnit.MILLISECONDS);

    public class robotController {

        public class Update implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.update(timer.seconds());
                if(debug) {
                    if(logging.hasExpired())
                    {
                        logging.reset();
                        String debugMsg = robot.getTelem(timer.seconds());
                        debugMsg += String.format("loopTime = %4.0f\n", loopTimer.milliseconds());
                        telemetry.addLine(debugMsg);
                        BotLog.logD("BSDbg", debugMsg);
                    }
                    loopTimer.reset();
                }
                telemetry.update();
                return true;
            }
        }

        public Action updateRobot() {
            return new Update();
        }

        public class OpenClaw implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setClawPos(3);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }

        public class CloseClaw implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setClawPos(1);
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class PivotDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECINTAKE.getVal());
                return false;
            }
        }

        public Action pivotDown() {
            return new PivotDown();
        }

        public class PivotUp implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPEC.getVal());
                return false;
            }
        }
        public Action pivotUp() {
            return new PivotUp();
        }

        public class DiffyIntake implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setDiffyPos(70,-90);
                return false;
            }
        }
        public Action diffyIntake() {
            return new DiffyIntake();
        }

        public class DiffyPlace implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setDiffyPos(0,90);
                return false;
            }
        }
        public Action diffyPlace() {
            return new DiffyPlace();
        }


        public class DiffyRelease implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECINTAKE.getVal());
                return false;
            }
        }
        public Action diffyRelease() {
            return new DiffyRelease();
        }

        public class LiftDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(0, timer.seconds());
                return !robot.mLift.closeEnough();
            }
        }

        public Action liftDown() {
            return new LiftDown();
        }

        public class LiftIntakeInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(2, timer.seconds());
                return false;
            }
        }

        public Action liftIntakeInstant() {
            return new LiftIntakeInstant();
        }


        public class LiftClearInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(Lift.LIFT_POS.SPECCLEAR.getVal(), timer.seconds());
                return false;
            }
        }

        public Action liftClearInstant() {
            return new LiftClearInstant();
        }

        public class RobotReset implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(Lift.LIFT_POS.DOWN.getVal(), timer.seconds());
                robot.mIntake.setExtendoOpenLoop(-0.4);
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal());
                robot.mDeposit.setDiffyPos(0,0);

                return false;
            }
        }

        public Action robotReset() {
            return new RobotReset();
        }



        public class LiftPlaceInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(1, timer.seconds());
                return false;
            }
        }

        public Action liftPlaceInstant() {
            return new LiftPlaceInstant();
        }


        public class ClearWall implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (WallTimer.seconds()>0.5){
                    robot.mDeposit.setDiffyPos(-50, 70);
                    robot.mLift.setTargetPos(Lift.LIFT_POS.SPECIMEN_PLACE.getVal(), timer.seconds());
                    return false;
                }
                if (WallTimer.seconds()>0.3) {
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPEC.getVal());

                }
                return true;
            }
        }

        public Action clearWall() {
            return new ClearWall();
        }

        public class IntakeReset implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                if (WallTimer.seconds()>0.6){
                    robot.mIntake.zerofinish(timer.seconds());
                    return false;
                }else if (WallTimer.seconds()>0.5){
                    robot.mIntake.setExtendoOpenLoop(0);
                    robot.mIntake.rezero();
                    robot.mLift.setTargetPos(Lift.LIFT_POS.SPECINTAKE.getVal(), timer.seconds());
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECINTAKE.getVal());
                    robot.mDeposit.setClawPos(3);
                    robot.mDeposit.setDiffyPos(70,-90);
                }else{
                    robot.mIntake.setExtendoOpenLoop(-0.5);
                }
                return true;
            }
        }

        public Action intakeReset() {
            return new IntakeReset();
        }

        public class ResetTimer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                WallTimer.reset();
                return false;
            }
        }

        public Action resetTimer() {
            return new ResetTimer();
        }

        public class LiftUpInstant implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(2, timer.seconds());
                return false;
            }
        }

        public Action liftUpInstant() {
            return new LiftUpInstant();
        }

        public class LiftDownInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(0, timer.seconds());
                return false;
            }
        }

        public Action liftDownInstant() {
            return new LiftDownInstant();
        }
    }

    @Override
    public void runOpMode() {
        timer = new ElapsedTime();
        robotController controller = new robotController();
        Pose2d initialPose = new Pose2d((24-7.25+10), 7.25, Math.toRadians(90));
        //claw is 1.5 inches to the right
        robot = new Robot(this, initialPose , hardwareMap);
        PinpointDrive drive = new PinpointDrive(this.hardwareMap, initialPose);

        // vision here that outputs position
        int visionOutputPosition = 1;

        Action push = drive.actionBuilder(initialPose)
                .stopAndAdd(new SequentialAction(
                        controller.liftIntakeInstant(),
                        controller.openClaw(),
                        controller.diffyIntake(),
                        controller.pivotDown()
                ))
                .splineToConstantHeading(new Vector2d(35,36), Math.toRadians(90), new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 95))
                .splineToConstantHeading(new Vector2d(35,49), Math.toRadians(90), new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 95))
                .splineToConstantHeading(new Vector2d(42,57), Math.toRadians(0), new TranslationalVelConstraint(30 ), new ProfileAccelConstraint(-40,40))
                .splineToConstantHeading(new Vector2d(48,49), Math.toRadians(-90), new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 95))
                .splineToConstantHeading(new Vector2d(48,23), Math.toRadians(-90),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 95))
                .strafeToLinearHeading(new Vector2d(48.01,23),Math.toRadians(90))
                .waitSeconds(0.01)
                .splineToConstantHeading(new Vector2d(48,49), Math.toRadians(90), new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 95))
                .splineToConstantHeading(new Vector2d(53,57), Math.toRadians(0), new TranslationalVelConstraint(30 ), new ProfileAccelConstraint(-40,40))
                .splineToConstantHeading(new Vector2d(58,49), Math.toRadians(-90), new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 95))
                .splineToConstantHeading(new Vector2d(58,23), Math.toRadians(-90),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 95))
                .strafeToLinearHeading(new Vector2d(58.01,23),Math.toRadians(90))

                .waitSeconds(0.01)
                .splineToConstantHeading(new Vector2d(58,49), Math.toRadians(90), new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .splineToConstantHeading(new Vector2d(60.2,56), Math.toRadians(0), new TranslationalVelConstraint(30 ), new ProfileAccelConstraint(-50,30))
                .splineToConstantHeading(new Vector2d(62.5,49), Math.toRadians(-90), new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 90))
                .splineToConstantHeading(new Vector2d(62.5,23), Math.toRadians(-90), new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 90))
                .strafeToLinearHeading(new Vector2d(63.01,23),Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 80))

                .waitSeconds(0.01)
                .splineToLinearHeading(new Pose2d(35,6.5, Math.toRadians(90)), Math.toRadians(270))
                .stopAndAdd(new SequentialAction(
                        controller.closeClaw()
                ))
                .waitSeconds(0.1)
                .stopAndAdd(new SequentialAction(
                        controller.resetTimer(),
                        controller.liftClearInstant()
                ))
                .waitSeconds(0.05)
                .build();
        Action sub1 = drive.actionBuilder(new Pose2d(36,6.5, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-2, 41.5 ), Math.toRadians(90),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-50, 95))
                .stopAndAdd(new SequentialAction(
                        controller.openClaw(),
                        controller.diffyPlace(),
                        controller.resetTimer()
                ))
                .build();

        Action human1 = drive.actionBuilder(new Pose2d(-2,41.5, Math.toRadians(90)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(36, 6.5), Math.toRadians(-90),  new TranslationalVelConstraint(40 ), new ProfileAccelConstraint(-40, 80))
                .stopAndAdd(new SequentialAction(
                        controller.closeClaw()
                ))
                .waitSeconds(0.1)
                .stopAndAdd(new SequentialAction(
                        controller.resetTimer(),
                        controller.liftClearInstant()
                ))
                .waitSeconds(0.05)

                .build();
        Action sub2 = drive.actionBuilder(new Pose2d(36,6.5, Math.toRadians(90)))
                .setReversed(false)

                .splineToConstantHeading(new Vector2d(-0.5, 41.5 ), Math.toRadians(90),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-50, 95))
                .stopAndAdd(new SequentialAction(
                        controller.openClaw(),
                        controller.diffyPlace(),
                        controller.resetTimer()
                ))
                .build();

        Action human2 = drive.actionBuilder(new Pose2d(-0.5,41.5, Math.toRadians(90)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(36, 6.5), Math.toRadians(-90),  new TranslationalVelConstraint(40 ), new ProfileAccelConstraint(-40, 80))
                .stopAndAdd(new SequentialAction(
                        controller.closeClaw()
                ))
                .waitSeconds(0.1)
                .stopAndAdd(new SequentialAction(
                        controller.resetTimer(),
                        controller.liftClearInstant()
                ))
                .waitSeconds(0.05)

                .build();
        Action sub3 = drive.actionBuilder(new Pose2d(36,6.5, Math.toRadians(90)))
                .setReversed(false)

                .splineToConstantHeading(new Vector2d(1, 41.5 ), Math.toRadians(90),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-50, 95))
                .stopAndAdd(new SequentialAction(
                        controller.openClaw(),
                        controller.diffyPlace(),
                        controller.resetTimer()
                ))
                .build();

        Action human3 = drive.actionBuilder(new Pose2d(1,41.5, Math.toRadians(90)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(36, 5.5), Math.toRadians(-90),  new TranslationalVelConstraint(40 ), new ProfileAccelConstraint(-40, 80))
                .stopAndAdd(new SequentialAction(
                        controller.closeClaw()
                ))
                .waitSeconds(0.1)
                .stopAndAdd(new SequentialAction(
                        controller.resetTimer(),
                        controller.liftClearInstant()
                ))
                .waitSeconds(0.05)

                .build();
        Action sub4 = drive.actionBuilder(new Pose2d(36,6, Math.toRadians(90)))
                .setReversed(false)

                .splineToConstantHeading(new Vector2d(2.5, 41.5 ), Math.toRadians(90),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-50, 95))
                .stopAndAdd(new SequentialAction(
                        controller.openClaw(),
                        controller.diffyPlace(),
                        controller.resetTimer()
                ))
                .build();

        Action human4 = drive.actionBuilder(new Pose2d(2.5,41.5, Math.toRadians(90)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(36, 6), Math.toRadians(-90),  new TranslationalVelConstraint(40 ), new ProfileAccelConstraint(-40, 80))
                .stopAndAdd(new SequentialAction(
                        controller.closeClaw()
                ))
                .waitSeconds(0.1)
                .stopAndAdd(new SequentialAction(
                        controller.resetTimer(),
                        controller.liftClearInstant()
                ))
                .waitSeconds(0.05)

                .build();
        Action sub5 = drive.actionBuilder(new Pose2d(36,6, Math.toRadians(90)))
                .setReversed(false)

                .splineToConstantHeading(new Vector2d(3.5, 41.5), Math.toRadians(90),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-50, 95))
                .stopAndAdd(new SequentialAction(
                        controller.openClaw(),
                        controller.diffyPlace(),
                        controller.resetTimer()
                ))
                .build();

        Action human5 = drive.actionBuilder(new Pose2d(3.5,41.5, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(40, 10), Math.toRadians(-45))
                .stopAndAdd(new SequentialAction(
                        controller.closeClaw()
                ))
                .waitSeconds(0.2)
                .stopAndAdd(new SequentialAction(
                        controller.resetTimer(),
                        controller.robotReset()
                ))

                .build();








        //.stopAndAdd()




        // actions that need to happen on init; for instance, a claw tightening.


        while (!isStopRequested() && !opModeIsActive()) {
            robot.update(timer.seconds());
            timer.reset();
            robot.autoInit();

        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();
        robot.mIntake.setExtendoPos(0, timer.seconds());

        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        controller.updateRobot(),
                        new SequentialAction(
                                push,
                                new ParallelAction(
                                        sub1,
                                        controller.clearWall()
                                ),
                                new ParallelAction(
                                        human1,
                                        controller.intakeReset()
                                ),
                                new ParallelAction(
                                        sub2,
                                        controller.clearWall()
                                ),
                                new ParallelAction(
                                        human2,
                                        controller.intakeReset()
                                ),
                                new ParallelAction(
                                        sub3,
                                        controller.clearWall()
                                ),
                                new ParallelAction(
                                        human3,
                                        controller.intakeReset()
                                ),
                                new ParallelAction(
                                        sub4,
                                        controller.clearWall()
                                ),
                                new ParallelAction(
                                        human4,
                                        controller.intakeReset()
                                ),
                                new ParallelAction(
                                        sub5,
                                        controller.clearWall()
                                ),
                                new ParallelAction(
                                        human5,
                                        controller.intakeReset()
                                )

                        )

                )
        );
        robot.stop();
    }
}