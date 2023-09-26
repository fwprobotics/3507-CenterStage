package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.pipelines.PropDetection;

public class FieldActionSequence {
    TrajectoryActionBuilder builder;
    Robot.AutoZoneColor autoZoneColor;
    Robot.AutoZoneHalf autoZoneHalf;
    Robot robot;
    public FieldActionSequence(TrajectoryActionBuilder builder, Robot.AutoZoneColor autoZoneColor, Robot.AutoZoneHalf autoZoneHalf, Robot robot) {
        this.builder = builder;
        this.autoZoneColor = autoZoneColor;
        this.autoZoneHalf = autoZoneHalf;
        this.robot = robot;
    }

    public FieldActionSequence dropPurplePixel(PropDetection.PropLocation propLocation) {
        builder = builder.strafeTo(new Vector2d((autoZoneHalf.xMult* 11)+ autoZoneHalf.xOffset, 35* autoZoneColor.yMult));
        switch (propLocation) {
            case LEFT:
                builder = builder
                        .turn(Math.toRadians(90))
                        .stopAndAdd(new SleepAction(1));
                break;
            case CENTER:
                builder = builder
                        .waitSeconds(1);
                break;
            case RIGHT:
                builder = builder
                        .strafeToLinearHeading(new Vector2d(31+ autoZoneHalf.xOffset, 35*autoZoneColor.yMult), Math.toRadians(180* autoZoneColor.yMult))
                        .waitSeconds(1);
                break;
        }
        builder = builder.stopAndAdd(
                new SequentialAction(
                        robot.arm.armStateAction(Arm.ArmState.PURPLEPIXEL),
                        new SleepAction(2),
                        robot.carousel.carouselStateAction(Carousel.CarouselStates.SLOT1, false)
                ));
        return this;
    }
    //left is -40 on blue 30 on red
    public FieldActionSequence dropYellowPixel(PropDetection.PropLocation propLocation) {
        builder = builder
                .strafeToLinearHeading(new Vector2d(32, 35*autoZoneColor.yMult), Math.toRadians(180* autoZoneColor.yMult))
                .stopAndAdd(robot.lift.liftStateAction(Lift.LiftState.UP))
                .splineToConstantHeading(new Vector2d(50, (((autoZoneColor.yMult >0) ? 40 : 30 )* autoZoneColor.yMult)-(propLocation.offset)), Math.toRadians(0));
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
