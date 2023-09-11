package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class FieldActionSequence {
    TrajectoryActionBuilder builder;
    Robot.AutoZoneColor autoZoneColor;
    Robot.AutoZoneHalf autoZoneHalf;
    public FieldActionSequence(TrajectoryActionBuilder builder, Robot.AutoZoneColor autoZoneColor, Robot.AutoZoneHalf autoZoneHalf) {
        this.builder = builder;
        this.autoZoneColor = autoZoneColor;
        this.autoZoneHalf = autoZoneHalf;
    }

    public FieldActionSequence dropPurplePixel(Robot.PropLocation propLocation) {
        builder = builder.strafeTo(new Vector2d(35* autoZoneColor.xMult, 11+ autoZoneHalf.yOffset));
        switch (propLocation) {
            case LEFT:
                builder = builder.turn(Math.toRadians(90)).stopAndAdd(new SleepAction(1));
                break;
            case CENTER:
                builder = builder.waitSeconds(1);
                break;
            case RIGHT:
                builder = builder.splineToLinearHeading(new Pose2d(35*autoZoneColor.xMult, 31+ autoZoneHalf.yOffset, Math.toRadians(90)), Math.toRadians(0)).waitSeconds(1);
                break;
        }
        return this;
    }

    public FieldActionSequence dropYellowPixel(Robot.PropLocation propLocation) {
        builder = builder.strafeToLinearHeading(new Vector2d(35*autoZoneColor.xMult, 24), Math.toRadians(-90)).splineTo(new Vector2d((((autoZoneColor.xMult <0) ? 40 : 30 )* autoZoneColor.xMult)+(propLocation.offset* autoZoneColor.xMult), 50), Math.toRadians(90));
        return this;
    }

    public FieldActionSequence park() {
        builder = builder.strafeTo(new Vector2d(60* autoZoneColor.xMult, 50)).strafeTo(new Vector2d(60* autoZoneColor.xMult, 60));
        return this;
    }

    public Action build() {
        return builder.build();
    }
}
