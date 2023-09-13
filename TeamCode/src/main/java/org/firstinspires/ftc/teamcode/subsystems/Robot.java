package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Robot {

    public enum AutoZoneColor {
        RED (1),
        BLUE (-1);

        int xMult;
        AutoZoneColor(int xMult) {
            this.xMult = xMult;
        }
    }

    public enum AutoZoneHalf {
        TOP(0),
        BOTTOM (-46);

        int yOffset;

        AutoZoneHalf(int offset) {
            yOffset = offset;
        }

    }
    AutoZoneColor startingZoneColor;
    AutoZoneHalf startingZoneHalf;
    MecanumDrive driveClass;
    public Robot(AutoZoneColor location, AutoZoneHalf autoZoneHalf, MecanumDrive driveClass) {
        startingZoneColor = location;
        this.startingZoneHalf = autoZoneHalf;
        this.driveClass = driveClass;
    }

    public FieldActionSequence createFieldActionSequence(Pose2d startingPos) {
        return new FieldActionSequence(driveClass.actionBuilder(startingPos), startingZoneColor, startingZoneHalf);
    }
}
