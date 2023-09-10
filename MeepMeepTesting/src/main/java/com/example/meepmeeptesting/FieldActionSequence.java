package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class FieldActionSequence {
    TrajectoryActionBuilder builder;

    public FieldActionSequence(TrajectoryActionBuilder builder) {
        this.builder = builder;
    }

    public FieldActionSequence dropPurplePixel(Robot.PropLocation propLocation) {
        builder = builder.splineTo(new Vector2d(35, -35), Math.toRadians(180));
        switch (propLocation) {
            case LEFT:
                builder = builder.turn(Math.toRadians(90)).stopAndAdd(new SleepAction(1));
                break;
            case CENTER:
                builder = builder.waitSeconds(1);
                break;
            case RIGHT:
                builder = builder.splineToLinearHeading(new Pose2d(35, -56, Math.toRadians(90)), Math.toRadians(0)).waitSeconds(1);
                break;
        }
        return this;
    }

    public FieldActionSequence dropYellowPixel(Robot.PropLocation propLocation) {
        builder = builder.splineToLinearHeading(new Pose2d(35, 24, Math.toRadians(90)), Math.toRadians(0));
        return this;
    }

    public Action build() {
        return builder.build();
    }
}
