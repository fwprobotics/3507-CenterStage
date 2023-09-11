package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Robot {
    enum PropLocation {
        LEFT (0),
        RIGHT (10),
        CENTER(5);

        int offset;
        PropLocation(int off) {
            offset = off;
        }
    }

    enum AutoZoneColor {
        RED (1),
        BLUE (-1);

        int xMult;
        AutoZoneColor(int xMult) {
            this.xMult = xMult;
        }
    }

    enum AutoZoneHalf {
        TOP(0),
        BOTTOM (-46);

        int yOffset;

        AutoZoneHalf(int offset) {
            yOffset = offset;
        }

    }
    AutoZoneColor startingZoneColor;
    AutoZoneHalf startingZoneHalf;
    Pose2d startingPos;
    RoadRunnerBotEntity driveClass;
    public Robot(AutoZoneColor location, AutoZoneHalf autoZoneHalf, RoadRunnerBotEntity driveClass) {
        startingZoneColor = location;
        this.startingZoneHalf = autoZoneHalf;
        this.driveClass = driveClass;
    }

    public FieldActionSequence createFieldActionSequence(Pose2d startingPos) {
        return new FieldActionSequence(driveClass.getDrive().actionBuilder(startingPos), startingZoneColor, startingZoneHalf);
    }
}
