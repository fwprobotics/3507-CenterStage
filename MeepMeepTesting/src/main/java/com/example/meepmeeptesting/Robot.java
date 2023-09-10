package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Robot {
    enum PropLocation {
        LEFT,
        RIGHT,
        CENTER,
        NONE
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
        TOP,
        BOTTOM;

    }
    AutoZoneColor startingZone;
    Pose2d startingPos;
    RoadRunnerBotEntity driveClass;
    public Robot(AutoZoneColor location, RoadRunnerBotEntity driveClass, Pose2d startingPos) {
        startingZone = location;
        this.startingPos = startingPos;
        this.driveClass = driveClass;
    }

    public FieldActionSequence createFieldActionSequence() {
        return new FieldActionSequence(driveClass.getDrive().actionBuilder(startingPos));
    }
}
