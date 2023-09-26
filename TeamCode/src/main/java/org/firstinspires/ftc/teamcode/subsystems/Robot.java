package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.MecanumDrive;

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
    AutoZoneColor startingZoneColor;
    AutoZoneHalf startingZoneHalf;
    MecanumDrive driveClass;
    public Arm arm;
    public Carousel carousel;
    public Lift lift;
    public Robot(AutoZoneColor location, AutoZoneHalf autoZoneHalf, MecanumDrive driveClass, Arm arm, Carousel carousel, Lift lift) {
        startingZoneColor = location;
        this.startingZoneHalf = autoZoneHalf;
        this.driveClass = driveClass;
        this.arm = arm;
        this.carousel = carousel;
        this.lift = lift;
    }

    public FieldActionSequence createFieldActionSequence(Pose2d startingPos) {
        return new FieldActionSequence(driveClass.actionBuilder(startingPos), startingZoneColor, startingZoneHalf, this);
    }
}
