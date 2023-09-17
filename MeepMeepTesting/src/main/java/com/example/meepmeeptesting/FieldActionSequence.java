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
        if ((propLocation == Robot.PropLocation.LEFT && autoZoneColor == Robot.AutoZoneColor.RED) || (propLocation == Robot.PropLocation.RIGHT && autoZoneColor == Robot.AutoZoneColor.BLUE))
        {
            builder = builder
                    .turn(Math.toRadians(90 * autoZoneColor.xMult))
                    .stopAndAdd(new SleepAction(1));
        } else if (propLocation == Robot.PropLocation.CENTER) {
            builder = builder
                    .waitSeconds(1);
        } else {
                builder = builder
                        .strafeToLinearHeading(new Vector2d(35*autoZoneColor.xMult, 31+ autoZoneHalf.yOffset), Math.toRadians(90* autoZoneColor.xMult))
                        .waitSeconds(1);
        }
        return this;
    }
    //left is -40 on blue 30 on red
    public FieldActionSequence dropYellowPixel(Robot.PropLocation propLocation) {
        builder = builder
                .strafeToLinearHeading(new Vector2d(35*autoZoneColor.xMult, 24), Math.toRadians(-90))
                .splineTo(new Vector2d((((autoZoneColor.xMult <0) ? 40 : 30 )* autoZoneColor.xMult)+(propLocation.offset), 50), Math.toRadians(90));
        return this;
    }

    public FieldActionSequence toStack() {
        builder = builder
                .strafeTo(new Vector2d(35* autoZoneColor.xMult, 24))
                .strafeTo(new Vector2d(60* autoZoneColor.xMult, -12))
                .strafeTo(new Vector2d(35* autoZoneColor.xMult, -55));
        return this;
    }

    public FieldActionSequence park() {
        builder = builder
                .strafeTo(new Vector2d(60* autoZoneColor.xMult, 50))
                .strafeTo(new Vector2d(60* autoZoneColor.xMult, 60));
        return this;
    }

    public Action build() {
        return builder.build();
    }
}
