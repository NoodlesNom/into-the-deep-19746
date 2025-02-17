package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.gf.OldAutoMaster.team;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.autonomous.rr.localizer.PinpointDrive;
import org.firstinspires.ftc.teamcode.util.BotLog;
import org.firstinspires.ftc.teamcode.util.StickyButton;

import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name = "6 spec 🚽🙍‍♂️", group = "Autonomous")


public class spec6 extends LinearOpMode {
    public Robot robot;
    public ElapsedTime timer;
    ElapsedTime generaltimer = new ElapsedTime();
    boolean debug = true;

    private boolean failed = true;
    private boolean genericboolean = false;
    public static int blockx = 2;
    public static int blocky = 2;

    private StickyButton intakeposup = new StickyButton();
    private StickyButton intakeposdown = new StickyButton();
    private StickyButton intakeposleft = new StickyButton();
    private StickyButton switchblock = new StickyButton();
    private StickyButton intakeposright = new StickyButton();
    ElapsedTime loopTimer = new ElapsedTime();
    Deadline logging = new Deadline(100, TimeUnit.MILLISECONDS);

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
                        debugMsg+="\nlift height: "+ robot.mLift.getLiftTicks();
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
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECINTAKE.getVal(), 900 , new double[] {1,2,2,2,2,2,2,1,1,1});
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
                robot.mDeposit.setDiffyPos(80,-90);
                return false;
            }
        }
        public Action diffyIntake() {
            return new DiffyIntake();
        }

        public class DiffyPlace implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setDiffyPos(90,90);
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOCLEAR.getVal());
                BotLog.logD("BSDbg", "END OF PLACING TRAJECTORY AHHHHHHHHHHH");
                return false;
            }
        }
        public Action diffyPlace() {
            return new DiffyPlace();
        }

        public class Pull implements Action {
            private boolean stopresetting = false;
            private boolean reject = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if ((robot.mIntake.detectedBlue()&&team.name().equals("RED"))||(robot.mIntake.detectedRed()&&team.name().equals("BLUE"))){
                    robot.mIntake.setIntakeOpenLoop(-0.8);
                    reject = true;
                }

                if (generaltimer.seconds()>1.7){

                    if(!reject){robot.mIntake.setIntakeOpenLoop(0);}
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                    robot.mIntake.setExtendoPos(0,timer.seconds());
                    return false;
                }else if(generaltimer.seconds()>1.6){
                    robot.mIntake.setOutputLimits(-1,1);
                    robot.mIntake.setClawPos(1);
                }else if(generaltimer.seconds()>1){

                    robot.mIntake.setExtendoTicks((int) (((blockx+5) * 25.4) / 1.25), timer.seconds());

                }else if(generaltimer.seconds()>0.6){
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                    if(!reject){robot.mIntake.setIntakeOpenLoop(1);}


                }else if(generaltimer.seconds()>0.1){
                    robot.mIntake.setExtendoTicks((int) (((blockx-2) * 25.4) / 1.25), timer.seconds());
                }else if(robot.mIntake.closeEnough()&&genericboolean){
                    if(!reject){robot.mIntake.setIntakeOpenLoop(0.5);}

                    stopresetting=true;
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.BLOCKCLEAR.getVal());
                }else if(!genericboolean&&robot.mIntake.getTargetExtendoIdx()!=Intake.EXTEND_POS.INTAKING.getVal()) {
                    robot.mIntake.setExtendoTicks((int) (((blockx) * 25.4) / 1.25), timer.seconds());

                    genericboolean = true;
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                    robot.mIntake.setOutputLimits(-1,0.5);
                }
                if (!stopresetting){
                    generaltimer.reset();
                }
                return true;
            }
        }

        public Action pull() {
            return new Pull();
        }


        public class DiffyPlaceLast implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setDiffyPos(90,90);
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECPUSH.getVal());
                BotLog.logD("BSDbg", "END OF PLACING TRAJECTORY AHHHHHHHHHHH");
                return false;
            }
        }
        public Action diffyPlaceLast() {
            return new DiffyPlaceLast();
        }

        public class subPrepareInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setClawPos(0);
                robot.mIntake.setExtendoTicks((int) ((blocky * 25.4) / 1.25), timer.seconds());
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                return false;
            }
        }

        public Action subPrepareInstant() {
            return new subPrepareInstant();
        }

        public class intakePrepareInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setClawPos(0);
                robot.mIntake.setExtendoPos(Intake.EXTEND_POS.AUTOINTAKEPREPARE.getVal(), timer.seconds());
                return false;
            }
        }

        public Action intakePrepareInstant() {
            return new intakePrepareInstant();
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

        public class Intakeing implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (robot.mLift.closeEnough()||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFERPREP.getVal()){
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if (generaltimer.seconds()>1.3){
                            robot.mIntake.setIntakeOpenLoop(0);
                            robot.mIntake.setClawPos(1);
                            return false;
                        }
                        else if (generaltimer.seconds()>1.1){
                            robot.mIntake.setClawPos(1);
                        }else if (generaltimer.seconds()>0.9){
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.BLOCKCLEAR.getVal());
                        }else if (generaltimer.seconds()>0){
                            robot.mIntake.setIntakeOpenLoop(1);
                            robot.mIntake.setClawPos(0);
                            robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), timer.seconds());
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                        }
                    }else{
                        generaltimer.reset();
                    }
                }else{
                    generaltimer.reset();
                }
                return true;
            }
        }


        public Action intakeing() {
            return new Intakeing();
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
                if (generaltimer.seconds()>1){
                    generaltimer.reset();
                }else if (generaltimer.seconds()>0.4){
                    robot.mDeposit.setDiffyPos(-40, 67);
                    robot.mLift.setTargetPos(Lift.LIFT_POS.SPECIMEN_PLACE.getVal(), timer.seconds());
                    return false;
                }else if (generaltimer.seconds()>0.2) {
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPEC.getVal(), 800 , new double[] {1,2,2,2,2,2,2,1,1,1});
                }else{
                    robot.mLift.setTargetPos(Lift.LIFT_POS.SPECCLEAR.getVal(), timer.seconds());
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
                if (generaltimer.seconds()>0.6){
                    robot.mIntake.zerofinish(timer.seconds());
                    return false;
                }else if (generaltimer.seconds()>0.5){
                    robot.mIntake.setExtendoOpenLoop(0);
                    robot.mIntake.rezero();
                    robot.mLift.setTargetPos(Lift.LIFT_POS.SPECINTAKE.getVal(), timer.seconds());
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECINTAKE.getVal(), 900 , new double[] {1,2,2,2,2,2,2,1,1,1});
                    robot.mDeposit.setClawPos(3);
                    robot.mDeposit.setDiffyPos(80,-90);
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
                generaltimer.reset();
                genericboolean = false;
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

        public class PivotShoot implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                return false;
            }
        }

        public Action pivotShoot() {
            return new PivotShoot();
        }

        public class Shoot implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (generaltimer.seconds()>0.3) {
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                    robot.mIntake.setIntakeOpenLoop(0);
                    return false;
                }else{
                    robot.mIntake.setIntakeOpenLoop(-0.9);
                }
                robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                if (!robot.mIntake.closeEnough()){
                    generaltimer.reset();
                }
                return true;
            }
        }

        public Action shoot() {
            return new Shoot();
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
        Pose2d initialPose = new Pose2d((10-7.25), 7.25, Math.toRadians(90));
        //claw is 1.5 inches to the right
        robot = new Robot(this, initialPose , hardwareMap);
        PinpointDrive drive = new PinpointDrive(this.hardwareMap, initialPose);

        // vision here that outputs position
        int visionOutputPosition = 1;



        Action sub1 = drive.actionBuilder(new Pose2d(36,6.5, Math.toRadians(90)))
                .setReversed(false)
                .setTangent(Math.toRadians(145))
                .splineToLinearHeading(new Pose2d(-3, 31 ,Math.toRadians(90)), Math.toRadians(145),  new TranslationalVelConstraint(50 ), new ProfileAccelConstraint(-40, 75))
                .splineToLinearHeading(new Pose2d(-5, 36 ,Math.toRadians(90)), Math.toRadians(90),  new TranslationalVelConstraint(50 ), new ProfileAccelConstraint(-40, 75))
                .splineToLinearHeading(new Pose2d(-5, 41 ,Math.toRadians(90)), Math.toRadians(90),  new TranslationalVelConstraint(50 ), new ProfileAccelConstraint(-40, 75))

                .stopAndAdd(new SequentialAction(
                        controller.openClaw(),
                        controller.diffyPlace(),
                        controller.resetTimer()
                ))
                .build();

        Action human1 = drive.actionBuilder(new Pose2d(-5,41, Math.toRadians(90)))
                .setReversed(true)
                .setTangent(Math.toRadians(90-180))
                .splineToLinearHeading(new Pose2d(33, 20 ,Math.toRadians(90)), Math.toRadians(145-180),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .splineToLinearHeading(new Pose2d(36, 12 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .splineToLinearHeading(new Pose2d(36, 6.5 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
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
                .setTangent(Math.toRadians(145))
                .splineToLinearHeading(new Pose2d(-1, 31 ,Math.toRadians(90)), Math.toRadians(145),  new TranslationalVelConstraint(50 ), new ProfileAccelConstraint(-40, 75))
                .splineToLinearHeading(new Pose2d(-3, 36 ,Math.toRadians(90)), Math.toRadians(90),  new TranslationalVelConstraint(50 ), new ProfileAccelConstraint(-40, 75))
                .splineToLinearHeading(new Pose2d(-3, 41 ,Math.toRadians(90)), Math.toRadians(90),  new TranslationalVelConstraint(50 ), new ProfileAccelConstraint(-40, 75))
                .stopAndAdd(new SequentialAction(
                        controller.openClaw(),
                        controller.diffyPlace(),
                        controller.resetTimer()
                ))
                .build();

        Action human2 = drive.actionBuilder(new Pose2d(-3,41, Math.toRadians(90)))
                .setReversed(true)
                .setTangent(Math.toRadians(90-180))
                .splineToLinearHeading(new Pose2d(33, 20 ,Math.toRadians(90)), Math.toRadians(145-180),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .splineToLinearHeading(new Pose2d(36, 12 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .splineToLinearHeading(new Pose2d(36, 6.5 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
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
                .setTangent(Math.toRadians(145))
                .splineToLinearHeading(new Pose2d(1, 31 ,Math.toRadians(90)), Math.toRadians(145),  new TranslationalVelConstraint(50 ), new ProfileAccelConstraint(-40, 75))
                .splineToLinearHeading(new Pose2d(-1, 36 ,Math.toRadians(90)), Math.toRadians(90),  new TranslationalVelConstraint(50 ), new ProfileAccelConstraint(-40, 75))
                .splineToLinearHeading(new Pose2d(-1, 41 ,Math.toRadians(90)), Math.toRadians(90),  new TranslationalVelConstraint(50 ), new ProfileAccelConstraint(-40, 75))
                .stopAndAdd(new SequentialAction(
                        controller.openClaw(),
                        controller.diffyPlace(),
                        controller.resetTimer()
                ))
                .build();

        Action human3 = drive.actionBuilder(new Pose2d(-1,41, Math.toRadians(90)))
                .setReversed(true)
                .setTangent(Math.toRadians(90-180))
                .splineToLinearHeading(new Pose2d(33, 20 ,Math.toRadians(90)), Math.toRadians(145-180),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .splineToLinearHeading(new Pose2d(36, 12 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .splineToLinearHeading(new Pose2d(36, 6.5 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
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
        Action sub4 = drive.actionBuilder(new Pose2d(36,6.5, Math.toRadians(90)))
                .setReversed(false)
                .setTangent(Math.toRadians(145))
                .splineToLinearHeading(new Pose2d(3, 31 ,Math.toRadians(90)), Math.toRadians(145),  new TranslationalVelConstraint(50 ), new ProfileAccelConstraint(-40, 75))
                .splineToLinearHeading(new Pose2d(1, 36 ,Math.toRadians(90)), Math.toRadians(90),  new TranslationalVelConstraint(50 ), new ProfileAccelConstraint(-40, 75))
                .splineToLinearHeading(new Pose2d(1, 41 ,Math.toRadians(90)), Math.toRadians(90),  new TranslationalVelConstraint(50 ), new ProfileAccelConstraint(-40, 75))
                .stopAndAdd(new SequentialAction(
                        controller.openClaw(),
                        controller.diffyPlace(),
                        controller.resetTimer()
                ))
                .build();

        Action human4 = drive.actionBuilder(new Pose2d(1,41, Math.toRadians(90)))
                .setReversed(true)
                .setTangent(Math.toRadians(90-180))
                .splineToLinearHeading(new Pose2d(33, 20 ,Math.toRadians(90)), Math.toRadians(145-180),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .splineToLinearHeading(new Pose2d(36, 12 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .splineToLinearHeading(new Pose2d(36, 6.5 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
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
        Action sub5 = drive.actionBuilder(new Pose2d(36,6.5, Math.toRadians(90)))
                .setReversed(false)
                .setTangent(Math.toRadians(145))
                .splineToLinearHeading(new Pose2d(5, 31 ,Math.toRadians(90)), Math.toRadians(145),  new TranslationalVelConstraint(50 ), new ProfileAccelConstraint(-40, 75))
                .splineToLinearHeading(new Pose2d(3, 36 ,Math.toRadians(90)), Math.toRadians(90),  new TranslationalVelConstraint(50 ), new ProfileAccelConstraint(-40, 75))
                .splineToLinearHeading(new Pose2d(3, 41 ,Math.toRadians(90)), Math.toRadians(90),  new TranslationalVelConstraint(50 ), new ProfileAccelConstraint(-40, 75))
                .stopAndAdd(new SequentialAction(
                        controller.openClaw(),
                        controller.diffyPlaceLast(),
                        controller.resetTimer()
                ))
                .build();

        Action human5 = drive.actionBuilder(new Pose2d(3,41, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(48, 12), Math.toRadians(-15),new TranslationalVelConstraint(90 ), new ProfileAccelConstraint(-100, 90))
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
            intakeposleft.update(gamepad1.dpad_left);
            intakeposright.update(gamepad1.dpad_right);
            intakeposup.update(gamepad1.dpad_up);
            intakeposdown.update(gamepad1.dpad_down);
            switchblock.update(gamepad1.x);
            robot.update(timer.seconds());
            timer.reset();
            robot.autoInit();
            robot.mDeposit.setLiveLed(team);
            telemetry.addLine("PRESS A (BOTTOM) FOR BLUE");
            telemetry.addLine("PRESS B (RIGHT) FOR RED");
            telemetry.addLine("CURRENT COLOR: " + team.name());
            if (gamepad1.a){
                team = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            }
            if (gamepad1.b){
                team = RevBlinkinLedDriver.BlinkinPattern.RED;
            }
            telemetry.addLine("USE DPAD TO ADJUST INTAKE POS BY INCHES");
            if (intakeposup.getState()){
                blocky +=1;
            }else if (intakeposdown.getState()&& blocky >0){
                blocky -=1;
            }else if (intakeposleft.getState()&& blockx >0){
                blockx -=1;
            }else if (intakeposright.getState()){
                blockx +=1;
            }

            telemetry.addLine("CURR INTAKE POS: "+ (blockx) + ", " + blocky);


            telemetry.update();


        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();
        robot.mIntake.setExtendoPos(0, timer.seconds());

        blockx+=5;

        Action preload = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(blockx, 41), Math.toRadians(90))
                .stopAndAdd(new SequentialAction(
                        controller.openClaw(),
                        controller.diffyPlace(),
                        controller.resetTimer()
                ))
                .build();
        Action shoot1 = drive.actionBuilder(new Pose2d(blockx, 41, Math.toRadians(90)))
                .setReversed(true)
                .afterTime(0.5,new SequentialAction(
                        controller.liftUpInstant(),
                        controller.pivotShoot()
                ))
                .splineToLinearHeading(new Pose2d(59,19, Math.toRadians(90)), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .stopAndAdd(new SequentialAction(
                        controller.resetTimer(),
                        controller.shoot()
                ))
                .build();
        Action intake1 = drive.actionBuilder(new Pose2d(59, 19, Math.toRadians(90)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(59,26, Math.toRadians(90)), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .build();

        Action shoot2 = drive.actionBuilder(new Pose2d(59, 26, Math.toRadians(90)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(62,19, Math.toRadians(70)), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .build();
        Action intake2 = drive.actionBuilder(new Pose2d(62, 19, Math.toRadians(70)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(64,22, Math.toRadians(70)), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .build();
        Action shoot3 = drive.actionBuilder(new Pose2d(64, 22, Math.toRadians(90)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(48,19, Math.toRadians(90)), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .build();
        Action intake3 = drive.actionBuilder(new Pose2d(48, 19, Math.toRadians(90)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(48,22, Math.toRadians(90)), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .build();
        Action wall = drive.actionBuilder(new Pose2d(48, 22, Math.toRadians(90)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(37,7, Math.toRadians(90)), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .build();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        controller.updateRobot(),
                        new SequentialAction(
                                preload,
                                controller.pull(),
                                shoot1,
                                new ParallelAction(
                                        intake1,
                                        controller.intakeing()
                                ),
                                new ParallelAction(
                                        shoot2,
                                        controller.shoot()
                                ),
                                new ParallelAction(
                                        intake2,
                                        controller.intakeing()
                                ),
                                new ParallelAction(
                                        shoot3,
                                        controller.shoot()
                                ),
                                new ParallelAction(
                                        intake3,
                                        controller.intakeing()
                                ),
                                wall,
                                new ParallelAction(
                                        sub1,
                                        controller.clearWall()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        human1,
                                        controller.intakeReset()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        sub2,
                                        controller.clearWall()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        human2,
                                        controller.intakeReset()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        sub3,
                                        controller.clearWall()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        human3,
                                        controller.intakeReset()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        sub4,
                                        controller.clearWall()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        human4,
                                        controller.intakeReset()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        sub5,
                                        controller.clearWall()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        human5,
                                        controller.liftDownInstant()
                                )

                        )

                )
        );
        robot.stop();
    }
}