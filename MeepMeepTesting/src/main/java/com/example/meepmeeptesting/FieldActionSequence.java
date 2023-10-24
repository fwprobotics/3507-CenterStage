package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
        //   builder = builder.strafeTo(new Vector2d((autoZoneHalf.xMult* 11)+ autoZoneHalf.xOffset, 35* autoZoneColor.yMult));
        if ((propLocation == Robot.PropLocation.LEFT && autoZoneColor == Robot.AutoZoneColor.RED) || (propLocation == Robot.PropLocation.RIGHT && autoZoneColor == Robot.AutoZoneColor.BLUE)) {
//            builder = builder
//                    .turn(Math.toRadians(-90 * autoZoneColor.yMult))
//                    .stopAndAdd(new SleepAction(1));
            builder = builder.strafeToLinearHeading(new Vector2d((7) + 2 * autoZoneHalf.xOffset, 30 * autoZoneColor.yMult), Math.toRadians(135 * -autoZoneColor.yMult));
        } else if (propLocation == Robot.PropLocation.CENTER) {
//            builder = builder
//                    .waitSeconds(1);
            builder = builder.strafeToLinearHeading(new Vector2d((autoZoneHalf.xMult * 12) + autoZoneHalf.xOffset, 36 * autoZoneColor.yMult), Math.toRadians(90 * -autoZoneColor.yMult));
        } else {
//                builder = builder
//                        .strafeToLinearHeading(new Vector2d(31+ autoZoneHalf.xOffset, 35*autoZoneColor.yMult), Math.toRadians(180* autoZoneColor.yMult))
//                        .waitSeconds(1);
            builder = builder.strafeToLinearHeading(new Vector2d((17) + 2 * autoZoneHalf.xOffset, 35 * autoZoneColor.yMult), Math.toRadians(45 * -autoZoneColor.yMult));
        }

        if (autoZoneHalf == Robot.AutoZoneHalf.RIGHT) {
            builder = builder.strafeToLinearHeading(new Vector2d(autoZoneHalf.xMult * 12 + autoZoneHalf.xOffset, 60 * autoZoneColor.yMult), Math.toRadians(180))
                    .strafeTo(new Vector2d(36, 60 * autoZoneColor.yMult));
        } else {
            builder = builder.strafeToLinearHeading(new Vector2d(autoZoneHalf.xMult * 12 + autoZoneHalf.xOffset, 60 * autoZoneColor.yMult), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(-57, 45* autoZoneColor.yMult), Math.toRadians(180))
                    .strafeToConstantHeading(new Vector2d(-57,12* autoZoneColor.yMult ))
                    .strafeToConstantHeading(new Vector2d(45, 12* autoZoneColor.yMult));
        }
        return this;
    }
    //left is -40 on blue 30 on red
    public FieldActionSequence dropYellowPixel(Robot.PropLocation propLocation) {
        builder = builder

                .strafeToConstantHeading(new Vector2d(54, (((autoZoneColor.yMult >0) ? 40 : 30 )* autoZoneColor.yMult)-(propLocation.offset)));
        return this;
    }

    public FieldActionSequence toStack() {
        builder = builder
                .splineToConstantHeading(new Vector2d(36, 60* autoZoneColor.yMult), Math.toRadians(180))
                .strafeTo(new Vector2d(-36, 60* autoZoneColor.yMult))
                .strafeTo(new Vector2d(-55, 35* autoZoneColor.yMult));
        return this;
    }

    public FieldActionSequence park() {
        builder = builder
                .strafeTo(new Vector2d(50, 60* autoZoneColor.yMult))
                .strafeTo(new Vector2d(60, 60* autoZoneColor.yMult));
        return this;
    }

    public Action build() {
        return builder.build();
    }
}
