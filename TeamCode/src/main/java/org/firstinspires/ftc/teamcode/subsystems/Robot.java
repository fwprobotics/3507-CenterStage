package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.FSMAction;

public class Robot {

    public enum AutoZoneColor {
        RED (-1),
        BLUE (1);

        int yMult;
        AutoZoneColor(int yMult) {
            this.yMult = yMult;
        }
    }

    public enum AutoZoneHalf {
        RIGHT(0, 1),
        LEFT (-24, -1);

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
    public Carousel carousel;
    public Lift lift;
    public Intake intake;
    public Robot(AutoZoneColor location, AutoZoneHalf autoZoneHalf, MecanumDrive driveClass, Arm arm, Carousel carousel, Lift lift, Intake intake) {
        startingZoneColor = location;
        this.startingZoneHalf = autoZoneHalf;
        this.driveClass = driveClass;
        this.arm = arm;
        this.carousel = carousel;
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
            boolean isNotDone = true;
            switch (state) {
                case INIT:
                    carousel.setState(Carousel.CarouselStates.SLOT1, true);
                    arm.setState(Arm.ArmState.INTAKE);
                    intake.setState(true);
                    timer.reset();
                    state = DISTATE.WAIT1;
                    break;
                case WAIT1:
                    if (timer.milliseconds() < 3000) { //replace with color sensor
                        carousel.setState(Carousel.CarouselStates.SLOT2, true);
                        timer.reset();
                        state = DISTATE.WAIT2;
                    }
                    break;
                case WAIT2:
                    if (timer.milliseconds() < 3000) {
                        carousel.setState(Carousel.CarouselStates.LOCK, true);
                        intake.setState(false);
                        isNotDone = false;
                    }
                    break;

            }
            return isNotDone;
        }
    }
}
