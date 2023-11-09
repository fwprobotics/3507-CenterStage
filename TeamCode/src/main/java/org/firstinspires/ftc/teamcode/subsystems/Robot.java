package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.FSMAction;

public class Robot {

    public enum AutoZoneColor {
        RED (-1, 0),
        BLUE (1, -2);

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
    public Robot(AutoZoneColor location, AutoZoneHalf autoZoneHalf, MecanumDrive driveClass, Arm arm, Claw claw, Lift lift, Intake intake) {
        startingZoneColor = location;
        this.startingZoneHalf = autoZoneHalf;
        this.driveClass = driveClass;
        this.arm = arm;
        this.claw = claw;
        this.lift = lift;
        this.intake = intake;
    }

    public FieldActionSequence createFieldActionSequence(Pose2d startingPos) {
        return new FieldActionSequence(driveClass.actionBuilder(startingPos), startingZoneColor, startingZoneHalf, this);
    }

    public Action doubleIntakeAction() {
        return new DoubleIntakeAction();

    }

    public class DoubleIntakeAction implements FSMAction {
        public DISTATE state;
        ElapsedTime timer;

        public DoubleIntakeAction() {
            state = DISTATE.INIT;
            timer = new ElapsedTime();
        }

        @Override
        public boolean stateMachine() {
            intake.setState(true);

            return claw.update(arm.currentState, Lift.LiftState.DOWN);
        }
    }
}
