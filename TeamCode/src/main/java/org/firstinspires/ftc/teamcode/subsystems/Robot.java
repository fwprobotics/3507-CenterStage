package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.FSMAction;

public class Robot {

    public enum AutoZoneColor {
        RED (-1, 0),
        BLUE (1, 0);

        int yMult;
        int yOffset;
        AutoZoneColor(int yMult, int yOffset) {
            this.yMult = yMult;
        }
    }

    public enum AutoZoneHalf {
        NEAR(0, 1),
        FAR (-24, -1);

        int xOffset;
        int xMult;

        AutoZoneHalf(int offset, int mult) {
            xOffset = offset;
            xMult = mult;
        }

    }

    public enum DISTATE {
        INIT,
        WAIT1,
        WAIT2
    }

    AutoZoneColor startingZoneColor;
    AutoZoneHalf startingZoneHalf;
    MecanumDrive driveClass;
    public Arm arm;
    public Claw claw;
    public Lift lift;
    public Intake intake;
    public Wacker wacker;
    public Robot(AutoZoneColor location, AutoZoneHalf autoZoneHalf, MecanumDrive driveClass, Arm arm, Claw claw, Lift lift, Intake intake, Wacker wacker) {
        startingZoneColor = location;
        this.startingZoneHalf = autoZoneHalf;
        this.driveClass = driveClass;
        this.arm = arm;
        this.claw = claw;
        this.lift = lift;
        this.intake = intake;
        this.wacker = wacker;
    }

    public FieldActionSequence createFieldActionSequence(Pose2d startingPos) {
        return new FieldActionSequence(driveClass.actionBuilder(startingPos), startingZoneColor, startingZoneHalf, this);
    }

    public Action doubleIntakeAction() {
        return telemetryPacket -> {
            intake.setState(true);
        claw.update(arm.currentState, Lift.LiftState.DOWN);
        boolean done = claw.clawRight.getPosition() == Claw.ClawPos.CLOSED.rightPos;

        if (done) {
            intake.setState(false);
            intake.manualControl(0, 1);
        }

        return !done;
        };

    }

//    public class DoubleIntakeAction implements FSMAction {
//        public DISTATE state;
//        ElapsedTime timer;
//
//        public DoubleIntakeAction() {
//            state = DISTATE.INIT;
//            timer = new ElapsedTime();
//        }
//
//        @Override
//        public boolean stateMachine() {
//            intake.setState(true);
//            boolean done = claw.update(arm.currentState, Lift.LiftState.DOWN);
//            if (done) intake.setState(false);
//            return done;
//        }
//    }

    public Action pixelMoverAction(Lift.LiftState liftState, Arm.ArmState armState) {
        if (lift.liftMotor.getCurrentPosition() < Lift.LiftState.LOW.height && armState != arm.currentState) {

            return new SequentialAction(
                    //lift.liftStateAction(Lift.LiftState.LOW),
                    arm.wristStateAction(armState),
                    new SleepAction(0.25),
                    arm.armStateAction(armState),

                    lift.liftStateAction(liftState)
            );
        }
        return new SequentialAction(
                arm.armStateAction(armState),
                arm.wristStateAction(armState),
                lift.liftStateAction(liftState)
        );
    }
}
