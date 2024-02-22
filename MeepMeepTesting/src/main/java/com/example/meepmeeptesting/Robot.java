package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Robot {
    enum PropLocation {
        LEFT (0),
        RIGHT (13),
        CENTER(6.5);

        double offset;
        PropLocation(double off) {
            offset = off;
        }
    }

    public enum AutoZoneColor {
        RED (-1, 0),
        BLUE (1, 0);

        int yMult;
        int yOffset;
        AutoZoneColor(int yMult, int yOffset) {
            this.yMult = yMult;
        }
    }

    enum AutoZoneHalf {
        NEAR(0, 1),
        FAR (-24, -1);

        int xOffset;
        int xMult;

        AutoZoneHalf(int offset, int mult) {
            xOffset = offset;
            xMult = mult;
        }

    }

    public enum AutoRoute {
        DEFAULT,
        THRU,
        LOOPFAR,
        LOOPMID,
        LOOPFARREV,
        LOOPMIDREV
    }

    AutoZoneColor startingZoneColor;
    AutoZoneHalf startingZoneHalf;
    AutoRoute autoRoute;
    RoadRunnerBotEntity driveClass;
    public Robot(AutoZoneColor location, AutoZoneHalf autoZoneHalf, AutoRoute autoRoute,  RoadRunnerBotEntity driveClass) {
        startingZoneColor = location;
        this.startingZoneHalf = autoZoneHalf;
        this.driveClass = driveClass;
        this.autoRoute = autoRoute;
    }

    public FieldActionSequence createFieldActionSequence(Pose2d startingPos) {
        return new FieldActionSequence(driveClass.getDrive().actionBuilder(startingPos), startingZoneColor, startingZoneHalf, autoRoute, this);
    }


}
