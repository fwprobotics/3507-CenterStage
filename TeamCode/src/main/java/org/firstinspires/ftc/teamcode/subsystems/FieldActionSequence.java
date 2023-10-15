package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmState.DROP;

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
        //   builder = builder.strafeTo(new Vector2d((autoZoneHalf.xMult* 11)+ autoZoneHalf.xOffset, 35* autoZoneColor.yMult));
        if ((propLocation == PropDetection.PropLocation.LEFT && autoZoneColor == Robot.AutoZoneColor.RED) || (propLocation == PropDetection.PropLocation.RIGHT && autoZoneColor == Robot.AutoZoneColor.BLUE))
        {
//            builder = builder
//                    .turn(Math.toRadians(-90 * autoZoneColor.yMult))
//                    .stopAndAdd(new SleepAction(1));
            builder = builder.strafeToLinearHeading(new Vector2d((7)+ 2*autoZoneHalf.xOffset, 30* autoZoneColor.yMult), Math.toRadians(135* -autoZoneColor.yMult));
        } else if (propLocation == PropDetection.PropLocation.CENTER) {
//            builder = builder
//                    .waitSeconds(1);
            builder = builder.strafeToLinearHeading(new Vector2d((autoZoneHalf.xMult* 12)+ autoZoneHalf.xOffset, 36* autoZoneColor.yMult), Math.toRadians(90*-autoZoneColor.yMult));
        } else {
//                builder = builder
//                        .strafeToLinearHeading(new Vector2d(31+ autoZoneHalf.xOffset, 35*autoZoneColor.yMult), Math.toRadians(180* autoZoneColor.yMult))
//                        .waitSeconds(1);
            builder = builder.strafeToLinearHeading(new Vector2d((17)+ 2*autoZoneHalf.xOffset, 35* autoZoneColor.yMult), Math.toRadians(45*-autoZoneColor.yMult));
        }
//        builder = builder.stopAndAdd(
//                new SequentialAction(
//                        robot.arm.armStateAction(Arm.ArmState.DROP),
//                        new SleepAction(1),
//                        robot.carousel.carouselStateAction(Carousel.CarouselStates.SLOT1, false),
//                        new SleepAction(1),
//                        robot.lift.liftStateAction(Lift.LiftState.PURPLE),
//                        new SleepAction(1)
//                ));

        builder = builder.strafeToLinearHeading(new Vector2d(autoZoneHalf.xMult*12+ autoZoneHalf.xOffset, 60* autoZoneColor.yMult), Math.toRadians(180))
                .strafeTo(new Vector2d(36, 60* autoZoneColor.yMult));
        return this;
    }
    //left is -40 on blue 30 on red
    public FieldActionSequence dropYellowPixel(PropDetection.PropLocation propLocation) {
        builder = builder

                .strafeToConstantHeading(new Vector2d(54, (((autoZoneColor.yMult >0) ? 40 : 30 )* autoZoneColor.yMult)-(propLocation.offset)))
                .stopAndAdd(
                        new SequentialAction(
                                robot.lift.liftStateAction(Lift.LiftState.MID),
                                new SleepAction(2),
                                robot.arm.armStateAction(DROP),
                                robot.carousel.carouselStateAction(Carousel.CarouselStates.SLOT2, false),
                            new SleepAction(1),
                            robot.lift.liftStateAction(Lift.LiftState.DOWN)));
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
