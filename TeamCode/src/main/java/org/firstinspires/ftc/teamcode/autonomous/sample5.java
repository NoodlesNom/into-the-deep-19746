package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.autonomous.rr.localizer.PinpointDrive;

@Config
@Autonomous(name = "5 sample 🥵", group = "Autonomous")


public class sample5 extends LinearOpMode {
    public Robot robot;
    public ElapsedTime timer;
    private boolean genericboolean = false;

    ElapsedTime generaltimer = new ElapsedTime();


    public class robotController {

        public class Update implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.update(timer.seconds());
                return true;
            }
        }

        public Action updateRobot() {
            return new Update();
        }

        public class OpenClaw implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setClawPos(2);
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
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(), 750, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                robot.mDeposit.setDiffyPos(25, -90);
                return false;
            }
        }

        public Action pivotDown() {
            return new PivotDown();
        }

        public class PivotIdle implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal(), 1000, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                return false;
            }
        }

        public Action pivotIdle() {
            return new PivotIdle();
        }

        public class PivotUp implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLE.getVal(), 500, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                return robot.mDeposit.servoDone();
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
                robot.mDeposit.setDiffyPos(0,-90);
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

        public class LiftUp implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                return !robot.mLift.closeEnough();
            }
        }

        public Action liftUp() {
            return new LiftUp();
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


        public class Transfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                robot.mDeposit.setDiffyPos(40, 90);
                if (robot.mIntake.closeEnoughAuto()&&robot.mLift.closeEnough()){
                    if (generaltimer.seconds()>0.7){
                        robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                    }else if (generaltimer.seconds()>0.6){
                        robot.mDeposit.setClawPos(1);
                    }else if (generaltimer.seconds()>0.4){
                        robot.mIntake.setIntakeOpenLoop(-0.8);
                        robot.mIntake.setExtendoOpenLoop(-0.6);
                    }
                }else if (robot.mLift.getLiftTargetPos() != Lift.LIFT_POS.AUTOSAMPLE.getVal()){
                    robot.mIntake.setIntakeOpenLoop(0);
                    robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal());
                    robot.mIntake.setGatePos(Intake.GATE_POS.CLAMP.getVal());
                    generaltimer.reset();
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                }

                if (generaltimer.seconds()>1&&robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.AUTOSAMPLE.getVal()){
                    robot.mIntake.setIntakeOpenLoop(0);
                    robot.mIntake.setExtendoPos(0,timer.seconds());
                    generaltimer.reset();
                    return false;
                }
                return true;
            }
        }

        public Action transfer() {
            return new Transfer();
        }


        public class Sample1andIntake2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (robot.mLift.closeEnough()||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFERPREP.getVal()){
                    if (!genericboolean&&robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.AUTOSAMPLE.getVal()&&robot.mDeposit.getPivotPos() != Deposit.PIVOT_POS.TRANSFER.getVal()) {
                        robot.mIntake.setExtendoPos(Intake.EXTEND_POS.AUTOINTAKEPREPARE.getVal(), timer.seconds());
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLE.getVal(), 800, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                        robot.mDeposit.setDiffyPos(0,-90);
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                    }
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if (generaltimer.seconds()>1.5){
                            robot.mIntake.setIntakeOpenLoop(0);
                            return false;
                        }else if (generaltimer.seconds()>0.4){
                            robot.mIntake.setIntakeOpenLoop(1);
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(),1000, new double[] {1,2,3,4,4,4,3,2,1,1});
                            robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), timer.seconds());
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                        }else if (generaltimer.seconds()>0.2){
                            robot.mDeposit.setDiffyPos(40,90);
                            genericboolean = true;
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(),900, new double[] {1,2,3,4,4,4,3,2,1,1});
                        }else if (generaltimer.seconds()>0.1){
                            robot.mDeposit.setClawPos(2);
                        }
                    }else{
                        generaltimer.reset();
                    }
                }else{
                    generaltimer.reset();
                    if (robot.mLift.getLiftTicks()>700){
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal(), 800, new double[] {1,2,3,4,4,4,3,2,1,1});

                    }
                    robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                }
                return true;
            }
        }


        public Action sample1andIntake2() {
            return new Sample1andIntake2();
        }

        public class Sample2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (robot.mLift.closeEnough()||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFERPREP.getVal()){
                    if (!genericboolean&&robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.AUTOSAMPLE.getVal()&&robot.mDeposit.getPivotPos() != Deposit.PIVOT_POS.TRANSFER.getVal()) {
                        robot.mIntake.setExtendoPos(Intake.EXTEND_POS.GETOUT.getVal(), timer.seconds());
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLE.getVal(), 800, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                        robot.mDeposit.setDiffyPos(0,-90);
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                    }
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if (generaltimer.seconds()>0.4){
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(),1000, new double[] {1,2,3,4,4,4,3,2,1,1});
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                            return false;
                        }else if (generaltimer.seconds()>0.2){
                            robot.mDeposit.setDiffyPos(40,90);
                            genericboolean = true;
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(),900, new double[] {1,2,3,4,4,4,3,2,1,1});
                        }else if (generaltimer.seconds()>0.1){
                            robot.mDeposit.setClawPos(2);
                        }
                    }else{
                        generaltimer.reset();
                    }
                }else{
                    generaltimer.reset();
                    if (robot.mLift.getLiftTicks()>700){
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal(), 800, new double[] {1,2,3,4,4,4,3,2,1,1});

                    }
                    robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                }
                return true;
            }
        }


        public Action sample2() {
            return new Sample2();
        }

        public class Sample3 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (robot.mLift.closeEnough()||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFERPREP.getVal()){
                    if (!genericboolean&&robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.AUTOSAMPLE.getVal()&&robot.mDeposit.getPivotPos() != Deposit.PIVOT_POS.TRANSFER.getVal()) {
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLE.getVal(), 800, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                        robot.mDeposit.setDiffyPos(0,-90);
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                    }
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if (generaltimer.seconds()>0.4){
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(),1000, new double[] {1,2,3,4,4,4,3,2,1,1});
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                            return false;
                        }else if (generaltimer.seconds()>0.2){
                            robot.mDeposit.setDiffyPos(40,90);
                            genericboolean = true;
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(),900, new double[] {1,2,3,4,4,4,3,2,1,1});
                        }else if (generaltimer.seconds()>0.1){
                            robot.mDeposit.setClawPos(2);
                        }
                    }else{
                        generaltimer.reset();
                    }
                }else{
                    generaltimer.reset();
                    if (robot.mLift.getLiftTicks()>700){
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal(), 800, new double[] {1,2,3,4,4,4,3,2,1,1});

                    }
                    robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                }
                return true;
            }
        }


        public Action sample3() {
            return new Sample3();
        }


        public class Intakeing implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (robot.mLift.closeEnough()||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFERPREP.getVal()){
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if (generaltimer.seconds()>1){
                            robot.mIntake.setIntakeOpenLoop(0);
                            return false;
                        }else if (generaltimer.seconds()>0){
                            robot.mIntake.setIntakeOpenLoop(1);
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(),1000, new double[] {1,2,3,4,4,4,3,2,1,1});
                            robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), timer.seconds());
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                        }
                    }else{
                        generaltimer.reset();
                    }
                }else{
                    generaltimer.reset();
                    if (robot.mLift.getLiftTicks()>700){
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal(), 800, new double[] {1,2,3,4,4,4,3,2,1,1});

                    }
                    robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                }
                return true;
            }
        }


        public Action intakeing() {
            return new Intakeing();
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
                robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                return false;
            }
        }

        public Action liftUpInstant() {
            return new LiftUpInstant();
        }

        public class LiftTransferPrepInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                return false;
            }
        }

        public Action liftTransferPrepInstant() {
            return new LiftTransferPrepInstant();
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

        public class ExtendoOutInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), timer.seconds());
                robot.mIntake.setIntakeOpenLoop(1);
                robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                return false;
            }
        }

        public Action extendoOutInstant() {
            return new ExtendoOutInstant();
        }

        public class ExtendoPrepareInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setExtendoPos(Intake.EXTEND_POS.AUTOINTAKEPREPARE.getVal(), timer.seconds());
                return false;
            }
        }

        public Action extendoPrepareInstant() {
            return new ExtendoPrepareInstant();
        }

        public class IntakeDownInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                return false;
            }
        }

        public Action intakeDownInstant() {
            return new IntakeDownInstant();
        }
    }

    @Override
    public void runOpMode() {
        timer = new ElapsedTime();
        robotController controller = new robotController();
        Pose2d initialPose = new Pose2d((24+7.25), 7.25, Math.toRadians(90));
        //claw is 1.5 inches to the right
        robot = new Robot(this, initialPose , hardwareMap);
        PinpointDrive drive = new PinpointDrive(this.hardwareMap, initialPose);

        // vision here that outputs position
        int visionOutputPosition = 1;

        Action preload_intake1 = drive.actionBuilder(initialPose)
                .stopAndAdd(new SequentialAction(
                        controller.liftUpInstant(),
                        controller.closeClaw(),
                        controller.diffyPlace(),
                        controller.pivotIdle()
                ))
                .strafeToLinearHeading(new Vector2d(11,18), Math.toRadians(70))
                .stopAndAdd(new SequentialAction(
                        controller.liftUp(),
                        controller.pivotUp(),
                        controller.extendoPrepareInstant(),
                        controller.intakeDownInstant()

                ))
                .waitSeconds(1)
                .stopAndAdd(new SequentialAction(
                        controller.openClaw()
                ))
                .waitSeconds(0.1)
                .stopAndAdd(new SequentialAction(
                        controller.pivotDown(),
                        controller.liftTransferPrepInstant(),
                        controller.extendoOutInstant()
                ))
                .strafeToLinearHeading(new Vector2d(12,20), Math.toRadians(70))
                .stopAndAdd(controller.extendoOutInstant())
                .waitSeconds(0.75)
                .build();

        Action sample1_intake2 = drive.actionBuilder(new Pose2d(12,20, Math.toRadians(70)))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(7,19), Math.toRadians(87))
                .build();

        Action intake4 = drive.actionBuilder(new Pose2d(7,19, Math.toRadians(87)))
                .strafeToLinearHeading(new Vector2d(8,19), Math.toRadians(94))
                .build();

        Action sample3 = drive.actionBuilder(new Pose2d(8,19, Math.toRadians(94)))
                .strafeToLinearHeading(new Vector2d(7,19), Math.toRadians(87))
                .build();


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
                                preload_intake1,
                                new ParallelAction(
                                        sample1_intake2,
                                        new SequentialAction(
                                                controller.resetTimer(),
                                                controller.transfer()
                                        )
                                ),
                                controller.resetTimer(),
                                controller.sample1andIntake2(),
                                controller.resetTimer(),
                                controller.transfer(),
                                controller.resetTimer(),
                                controller.sample2(),
                                intake4,
                                controller.resetTimer(),
                                controller.intakeing(),
                                new ParallelAction(
                                        sample3,
                                        new SequentialAction(
                                                controller.resetTimer(),
                                                controller.transfer()
                                        )
                                ),
                                sample3






                        )

                )
        );
        robot.stop();
    }
}