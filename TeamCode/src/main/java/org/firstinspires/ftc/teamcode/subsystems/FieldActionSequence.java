package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmState.DRIVE;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmState.DROP;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmState.INTAKE;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmState.LIMBO;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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
        if ((propLocation == PropDetection.PropLocation.LEFT) )
        {
//            builder = builder
//                    .turn(Math.toRadians(-90 * autoZoneColor.yMult))
//                    .stopAndAdd(new SleepAction(1));

            builder = builder
                    .setReversed(true)
//                    .strafeTo(new Vector2d((autoZoneHalf.xMult* 14)+ autoZoneHalf.xOffset, 48* autoZoneColor.yMult))
//                    .strafeToLinearHeading(new Vector2d((autoZoneHalf.xMult* 36) + autoZoneHalf.xOffset, 32* autoZoneColor.yMult), Math.toRadians(180))
//                    .strafeTo(new Vector2d((autoZoneHalf.xMult* 10) + autoZoneHalf.xOffset, 32* autoZoneColor.yMult))
//                    .strafeTo(new Vector2d((autoZoneHalf.xMult* 14) + autoZoneHalf.xOffset, 32* autoZoneColor.yMult));
                    //                .strafeToLinearHeading(new Vector2d((autoZoneHalf.xMult* 14)+ autoZoneHalf.xOffset, 50* autoZoneColor.yMult), Math.toRadians(30* autoZoneColor.yMult))
                    //.splineToConstantHeading(new Vector2d((autoZoneHalf.xMult* 17)+ autoZoneHalf.xOffset, 45* autoZoneColor.yMult), Math.toRadians(0* autoZoneColor.yMult))
                    //bad on blue:
                    //.splineToLinearHeading(new Pose2d((autoZoneHalf.xMult* 14)+ autoZoneHalf.xOffset, 60* autoZoneColor.yMult, Math.toRadians(30* autoZoneColor.yMult)), Math.toRadians(-90* autoZoneColor.yMult))
                    .strafeTo(new Vector2d(autoZoneHalf.xMult*(14+ (autoZoneColor == Robot.AutoZoneColor.BLUE ? Math.cos(Math.toRadians(45))*7 : 0))+ autoZoneHalf.xOffset, (48 - (autoZoneColor.yMult > 0 ? Math.cos(Math.toRadians(45))*7 : 0))* autoZoneColor.yMult))
                    .splineToLinearHeading(new Pose2d(autoZoneHalf.xMult*6+ autoZoneHalf.xOffset, (38 * autoZoneColor.yMult),  Math.toRadians(45* autoZoneColor.yMult + (autoZoneColor == Robot.AutoZoneColor.BLUE ? 90 : 0))), Math.toRadians(180 * autoZoneColor.yMult))
                    .strafeTo(new Vector2d((10)+ 2*autoZoneHalf.xOffset, 42* autoZoneColor.yMult))
                    .stopAndAdd(new SleepAction(0));
        } else if (propLocation == PropDetection.PropLocation.CENTER) {
//            builder = builder
//                    .waitSeconds(1);
            builder = builder.strafeToLinearHeading(new Vector2d((autoZoneHalf.xMult* 12)+ autoZoneHalf.xOffset, 35* autoZoneColor.yMult), Math.toRadians(90*autoZoneColor.yMult))
                    .strafeTo(new Vector2d((autoZoneHalf.xMult* 12)+ autoZoneHalf.xOffset, 38* autoZoneColor.yMult));
        } else {
//                builder = builder
//                        .strafeToLinearHeading(new Vector2d(31+ autoZoneHalf.xOffset, 35*autoZoneColor.yMult), Math.toRadians(180* autoZoneColor.yMult))
//                        .waitSeconds(1);
            //old: 17, 43
            builder = builder
                    .strafeToLinearHeading(new Vector2d((((autoZoneColor == Robot.AutoZoneColor.BLUE? 8 : 15) + 2*autoZoneHalf.xOffset)), ((33 + (autoZoneColor.yMult > 0 ? Math.cos(Math.toRadians(45))*7 : 0)) * autoZoneColor.yMult)), Math.toRadians(135*autoZoneColor.yMult - (autoZoneColor == Robot.AutoZoneColor.BLUE ? 90 : 0)))
                    .strafeTo(new Vector2d((((autoZoneColor == Robot.AutoZoneColor.BLUE? 11 : 12) + 2*autoZoneHalf.xOffset)), ((38 + (autoZoneColor.yMult > 0 ? Math.cos(Math.toRadians(45))*7 : 0)) * autoZoneColor.yMult)));
//            builder = builder.setReversed(true)
//                    .splineToLinearHeading(new Pose2d((15)+ 2*autoZoneHalf.xOffset, 38* autoZoneColor.yMult, Math.toRadians(135*autoZoneColor.yMult)), Math.toRadians(-45*autoZoneColor.yMult));
            //     builder = builder.strafeTo(new Vector2d(24, 38));
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
//        builder = builder.stopAndAdd(new SequentialAction(robot.intake.intakeRunAction(-10),
//                new SleepAction(2)));

        if (autoZoneHalf == Robot.AutoZoneHalf.NEAR) {
            builder = builder.strafeToLinearHeading(new Vector2d(autoZoneHalf.xMult * 12 + autoZoneHalf.xOffset, 48 * autoZoneColor.yMult), Math.toRadians(180));
            //   .splineToConstantHeading(new Vector2d(35, 45 * autoZoneColor.yMult), Math.toRadians(0));
            //     .splineToConstantHeading(new Vector2d(35, 36* autoZoneColor.yMult), Math.toRadians(-90* autoZoneColor.yMult));

        } else {
            builder = builder.splineToLinearHeading(new Pose2d(autoZoneHalf.xMult * 12 + autoZoneHalf.xOffset, 50 * autoZoneColor.yMult, Math.toRadians(180)), Math.toRadians(180))
                    .strafeTo(new Vector2d(-57, 50* autoZoneColor.yMult))
                    .strafeTo(new Vector2d(-55,12* autoZoneColor.yMult))

                           .stopAndAdd(new SequentialAction( new SleepAction(0.5),
                                  robot.wacker.wackerAction(Wacker.WackerStates.TOP),
                                   new SleepAction(1)

                           ))
                    .strafeTo(new Vector2d(-59,12* autoZoneColor.yMult))
                    .strafeTo(new Vector2d(-59,0* autoZoneColor.yMult))
                    .strafeTo(new Vector2d(-57,0* autoZoneColor.yMult))
                    .stopAndAdd(new SequentialAction( new SleepAction(0.5),
                            robot.wacker.wackerAction(Wacker.WackerStates.UP),
                            robot.intake.intakeAction(1)

                    ))
                    .strafeTo(new Vector2d(-66,8* autoZoneColor.yMult))
                    .stopAndAdd(
                            robot.doubleIntakeAction()
                    )
                    .splineToConstantHeading(new Vector2d(30, 12* autoZoneColor.yMult), Math.toRadians(0))
                    .stopAndAdd(robot.intake.intakeAction(0))
                    .strafeToLinearHeading(new Vector2d(36, 36 * autoZoneColor.yMult), Math.toRadians(180));
//                    .stopAndAdd(robot.arm.armStateAction(DRIVE));
        }

        builder = builder.stopAndAdd(
                new SequentialAction(
                robot.pixelMoverAction(Lift.LiftState.LOW, DROP),
                new SleepAction(2))
                );
        return this;
    }
    //left is -40 on blue 30 on red
    public FieldActionSequence dropYellowPixel(PropDetection.PropLocation propLocation) {
        builder = builder.setReversed(true)
                .splineToConstantHeading(new Vector2d(49.5, (((autoZoneColor.yMult >0) ? 44 : 30 )* autoZoneColor.yMult)-(propLocation.offset- autoZoneColor.yOffset)), Math.toRadians(0))
                .stopAndAdd(
                        new SequentialAction(
                            new SleepAction(1),
                            robot.claw.clawAction(Claw.ClawPos.OPEN, Claw.Claws.BOTH),
                            new SleepAction(2)));
        return this;
    }

    public FieldActionSequence park(PropDetection.PropLocation propLocation) {
        if (autoZoneHalf == Robot.AutoZoneHalf.NEAR) {
            builder = builder
                    .strafeTo(new Vector2d(40, (((autoZoneColor.yMult >0) ? 44 : 30 )* autoZoneColor.yMult)-(propLocation.offset- autoZoneColor.yOffset)))
                    .stopAndAdd(robot.pixelMoverAction(Lift.LiftState.DOWN, INTAKE))
                    .strafeTo(new Vector2d(40, 63* autoZoneColor.yMult))
                    .strafeTo(new Vector2d(60, 63 * autoZoneColor.yMult));
        }
        return this;
    }

    public FieldActionSequence toStack(int cycle) {
        builder = builder
                .splineToConstantHeading(new Vector2d(12, 12* autoZoneColor.yMult), Math.toRadians(180))
                .stopAndAdd(new SequentialAction(robot.pixelMoverAction(Lift.LiftState.DOWN, INTAKE), new SleepAction(3)))
                .splineToConstantHeading(new Vector2d(-50, 12* autoZoneColor.yMult), Math.toRadians(180))
                .stopAndAdd(
                        robot.wacker.wackerAction(Wacker.WackerStates.SECOND)
                )
                .splineToConstantHeading(new Vector2d(-57, (12+ cycle*12)* autoZoneColor.yMult), Math.toRadians(180))
                .strafeTo(new Vector2d(-57, (8+ cycle*12)* autoZoneColor.yMult))
                .strafeTo(new Vector2d(-55, (8+ cycle*12)* autoZoneColor.yMult))
                .stopAndAdd(robot.wacker.wackerAction(Wacker.WackerStates.UP))
                .stopAndAdd(new ParallelAction(
                        robot.doubleIntakeAction(),
                        robot.driveClass.actionBuilder(new Pose2d(-60, (8+ cycle*12)* autoZoneColor.yMult, Math.toRadians(180)))
                                .strafeTo(new Vector2d(-60, (8+ cycle*12)* autoZoneColor.yMult))
                                .build()
                ));
        return this;
    }

    public FieldActionSequence toBackDrop(int cycle) {
        builder = builder
                .splineToConstantHeading(new Vector2d(-50, 12* autoZoneColor.yMult), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(12, 12* autoZoneColor.yMult), Math.toRadians(0))
                .stopAndAdd(new SequentialAction( robot.pixelMoverAction(Lift.LiftState.LOW, DROP), new SleepAction(1)))
                .splineToConstantHeading(new Vector2d(54, (30 )* autoZoneColor.yMult), Math.toRadians(8))
                .stopAndAdd(new SequentialAction(
                        new SleepAction(1),
                        robot.claw.clawAction(Claw.ClawPos.OPEN, Claw.Claws.BOTH)
                ));
        return this;
    }

    public Action build() {
        return builder.build();
    }
}
