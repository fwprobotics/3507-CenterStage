package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmState.DRIVE;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmState.DROP;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmState.INTAKE;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmState.LIMBO;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.AutoZoneColor.BLUE;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.AutoZoneColor.RED;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.teamcode.pipelines.PropDetection;

public class FieldActionSequence {
    TrajectoryActionBuilder builder;
    Robot.AutoZoneColor autoZoneColor;
    Robot.AutoZoneHalf autoZoneHalf;
    Robot.AutoRoute autoRoute;
    Robot.AutoPark autoPark;
    Robot robot;
    public FieldActionSequence(TrajectoryActionBuilder builder, Robot.AutoZoneColor autoZoneColor, Robot.AutoZoneHalf autoZoneHalf, Robot.AutoRoute autoRoute, Robot.AutoPark autoPark,Robot robot) {
        this.builder = builder;
        this.autoZoneColor = autoZoneColor;
        this.autoZoneHalf = autoZoneHalf;
        this.autoRoute = autoRoute;
        this.autoPark = autoPark;
        this.robot = robot;
    }

    public FieldActionSequence dropPurplePixel(PropDetection.PropLocation propLocation) {
        if ((propLocation == PropDetection.PropLocation.LEFT) && ((autoZoneColor == BLUE && autoZoneHalf== Robot.AutoZoneHalf.FAR) || (autoZoneColor == RED && autoZoneHalf == Robot.AutoZoneHalf.NEAR)))
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
                    .strafeTo(new Vector2d(autoZoneHalf.xMult*(14+ (autoZoneColor == BLUE ? Math.cos(Math.toRadians(45))*7 : 0))+ autoZoneHalf.xOffset, (48 - (autoZoneColor.yMult > 0 ? Math.cos(Math.toRadians(45))*7 : 0))* autoZoneColor.yMult))
                    .splineToLinearHeading(new Pose2d(autoZoneHalf.xMult*5+ autoZoneHalf.xOffset, (38 * autoZoneColor.yMult),  Math.toRadians(45* autoZoneColor.yMult + (autoZoneColor == BLUE ? 90 : 0))), Math.toRadians(180 * autoZoneColor.yMult))
                    .strafeTo(new Vector2d((10)+ 2*autoZoneHalf.xOffset, 42* autoZoneColor.yMult))
                    .stopAndAdd(new SleepAction(0));

        } else if (propLocation == PropDetection.PropLocation.LEFT) {
            builder = builder
                    .strafeToLinearHeading(new Vector2d(autoZoneHalf.xMult*(23)+ autoZoneHalf.xOffset,((39 + (autoZoneColor.yMult > 0 ? Math.cos(Math.toRadians(45))*7 : 0)) * autoZoneColor.yMult)), Math.toRadians(45*autoZoneColor.yMult + (autoZoneColor == BLUE ? 90 : 0)))
                    .strafeTo(new Vector2d(autoZoneHalf.xMult*(16)+ autoZoneHalf.xOffset, ((37 + (autoZoneColor.yMult > 0 ? Math.cos(Math.toRadians(45))*7 : 0)) * autoZoneColor.yMult)));;
        } else if (propLocation == PropDetection.PropLocation.CENTER) {
//            builder = builder
//                    .waitSeconds(1);
            builder = builder.strafeToLinearHeading(new Vector2d((autoZoneHalf.xMult* 12)+ autoZoneHalf.xOffset, 34* autoZoneColor.yMult), Math.toRadians(90*autoZoneColor.yMult))
                    .strafeTo(new Vector2d((autoZoneHalf.xMult* 12)+ autoZoneHalf.xOffset, 38* autoZoneColor.yMult));
        } else {
//                builder = builder
//                        .strafeToLinearHeading(new Vector2d(31+ autoZoneHalf.xOffset, 35*autoZoneColor.yMult), Math.toRadians(180* autoZoneColor.yMult))
//                        .waitSeconds(1);
            //old: 17, 43
            builder = builder
                    .strafeToLinearHeading(new Vector2d((((autoZoneColor == BLUE? 8 : 15) + 2*autoZoneHalf.xOffset)), ((33 + (autoZoneColor.yMult > 0 ? Math.cos(Math.toRadians(45))*7 : 0)) * autoZoneColor.yMult)), Math.toRadians(135*autoZoneColor.yMult - (autoZoneColor == BLUE ? 90 : 0)))
                    .strafeTo(new Vector2d((((autoZoneColor == BLUE? 11 : 12) + 2*autoZoneHalf.xOffset)), ((38 + (autoZoneColor.yMult > 0 ? Math.cos(Math.toRadians(45))*7 : 0)) * autoZoneColor.yMult)));
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
        builder = builder.stopAndAdd(robot.lights.lightState(Lights.LightStates.PURPLE));
        if (autoZoneHalf == Robot.AutoZoneHalf.NEAR) {
            builder = builder.strafeToLinearHeading(new Vector2d(autoZoneHalf.xMult * 12 + autoZoneHalf.xOffset, 48 * autoZoneColor.yMult), Math.toRadians(180));
            //   .splineToConstantHeading(new Vector2d(35, 45 * autoZoneColor.yMult), Math.toRadians(0));
            //     .splineToConstantHeading(new Vector2d(35, 36* autoZoneColor.yMult), Math.toRadians(-90* autoZoneColor.yMult));

        } else {
            builder = builder.splineToLinearHeading(new Pose2d(autoZoneHalf.xMult * 12 + autoZoneHalf.xOffset, 50 * autoZoneColor.yMult, Math.toRadians(180)), Math.toRadians(180))
                    .strafeTo(new Vector2d(-59, 50* autoZoneColor.yMult))
                    .stopAndAdd(new SequentialAction(robot.flippers.singleFlipperAction(Flippers.FlipperState.OPEN, autoZoneColor == BLUE), new SleepAction(1)))
                    .strafeTo(new Vector2d(-59,(18* autoZoneColor.yMult)))

//                           .stopAndAdd(new SequentialAction(robot.driveClass.updateHeadingFromIMU(autoZoneColor == RED ? -90 : 90), new SleepAction(0.5)
//
//                           ))
                    .strafeTo(new Vector2d(-60,21* autoZoneColor.yMult))
                    .stopAndAdd(new SequentialAction(
                            robot.intake.intakeAction(1),
                            robot.flippers.singleFlipperAction(Flippers.FlipperState.CLOSED, autoZoneColor == BLUE),
                            robot.pixelIntakeAction(autoZoneColor == BLUE ? Claw.Claws.RIGHT: Claw.Claws.LEFT),
                            new SleepAction(0.5),
                            robot.lights.lightState(Lights.LightStates.PICKUP),
                            robot.driveClass.updateHeadingFromIMU(autoZoneColor == RED ? -90 : 90)



                    ))
//                    .strafeTo(new Vector2d(-57, (autoZoneColor == BLUE ? 6 : 4)* autoZoneColor.yMult))
//                    .stopAndAdd(
//                            new SequentialAction(
//                            robot.pixelIntakeAction(autoZoneColor == BLUE ? Claw.Claws.RIGHT: Claw.Claws.LEFT),
//                                    new SleepAction(0.5),
//                                    robot.driveClass.updateHeadingFromIMU(autoZoneColor == RED ? -90 : 90)
//                            )
//                    )
                    .setReversed(true)
                    .strafeToConstantHeading(new Vector2d(-36, 8* autoZoneColor.yMult))
                    .strafeToConstantHeading(new Vector2d(30, 8* autoZoneColor.yMult))
                 //   .stopAndAdd(robot.intake.intakeAction(0))
                    .strafeToLinearHeading(new Vector2d(36, 36 * autoZoneColor.yMult), Math.toRadians(180));
//                    .stopAndAdd(robot.arm.armStateAction(DRIVE));
        }

        builder = builder.stopAndAdd(
                new SequentialAction(
                robot.pixelMoverAction(Lift.LiftState.LOW, DROP),
                        robot.driveClass.updateHeadingFromIMU(autoZoneColor == RED ? -90 : 90),
                new SleepAction(1.5))
                );
        return this;
    }
    //left is -40 on blue 30 on red
    public FieldActionSequence dropYellowPixel(PropDetection.PropLocation propLocation) {
        builder = builder.setReversed(true)
                .strafeToConstantHeading(new Vector2d(49.5, (((autoZoneColor.yMult >0) ? 44 : 33 )* autoZoneColor.yMult)-(propLocation.offset- autoZoneColor.yOffset)))
                .stopAndAdd(
                        new SequentialAction(
                            new SleepAction(0.5),
                            robot.claw.clawAction(Claw.ClawPos.OPEN, Claw.Claws.BOTH),
                            robot.lights.lightState(Lights.LightStates.YELLOW),
//                                robot.driveClass.updateHeadingFromIMU(autoZoneColor == RED ? -90 : 90),
                            new SleepAction(0.5)));
        return this;
    }

    public FieldActionSequence park(PropDetection.PropLocation propLocation) {
        if (autoPark == Robot.AutoPark.WALL) {
            builder = builder
                    .strafeTo(new Vector2d(40, (((autoZoneColor.yMult >0) ? 44 : 30 )* autoZoneColor.yMult)-(propLocation.offset- autoZoneColor.yOffset)))
                    .stopAndAdd(robot.pixelMoverAction(Lift.LiftState.DOWN, INTAKE))
                    .strafeTo(new Vector2d(40, 63* autoZoneColor.yMult))
                    .strafeTo(new Vector2d(60, 63 * autoZoneColor.yMult));
        } else {
            builder = builder
                    .strafeTo(new Vector2d(40, (((autoZoneColor.yMult >0) ? 44 : 30 )* autoZoneColor.yMult)-(propLocation.offset- autoZoneColor.yOffset)))
                    .stopAndAdd(robot.pixelMoverAction(Lift.LiftState.DOWN, INTAKE))
                    .strafeTo(new Vector2d(40, 10* autoZoneColor.yMult))
                    .strafeTo(new Vector2d(60, 10 * autoZoneColor.yMult));
        }
        return this;
    }

    public FieldActionSequence toStack(int cycle) {
        switch (autoRoute) {
            case THRU:
            case LOOPMIDREV:
                builder =  builder.splineToConstantHeading(new Vector2d(36, 36* autoZoneColor.yMult), Math.toRadians(180))
                        .stopAndAdd( new SequentialAction(robot.pixelMoverAction(Lift.LiftState.DOWN, INTAKE), robot.driveClass.updateHeadingFromIMU(autoZoneColor == RED ? -90 : 90),new SleepAction(1)))

                        .splineToConstantHeading(new Vector2d(12, 36* autoZoneColor.yMult), Math.toRadians(180))

                        .splineToConstantHeading(new Vector2d(-36, 36* autoZoneColor.yMult), Math.toRadians(180))
                        .afterTime(0.5,
                                new SequentialAction(
                                        robot.flippers.flipperAction(Flippers.FlipperState.OPEN),
                                        robot.driveClass.updateHeadingFromIMU(autoZoneColor == RED ? -90 : 90)) //TODO: time to be saved
                        )
                        .splineToConstantHeading(new Vector2d(-58.5, 30* autoZoneColor.yMult), Math.toRadians(180))
                        .stopAndAdd(new SequentialAction(
                                robot.intake.intakeAction(1),
                                robot.flippers.flipperAction(Flippers.FlipperState.CLOSED),
                                robot.doubleIntakeAction(),
                                new SleepAction(0.5),
                                robot.lights.lightState(Lights.LightStates.PICKUP),
                                robot.driveClass.updateHeadingFromIMU(autoZoneColor == RED ? -90 : 90)



                        ));
                ;
                break;
            case LOOPFARREV:
                builder =  builder.splineToConstantHeading(new Vector2d(36, 60* autoZoneColor.yMult), Math.toRadians(180))
                        .stopAndAdd( new SequentialAction(robot.pixelMoverAction(Lift.LiftState.DOWN, INTAKE), robot.driveClass.updateHeadingFromIMU(autoZoneColor == RED ? -90 : 90),new SleepAction(1)))
                        .splineToConstantHeading(new Vector2d(12, 60* autoZoneColor.yMult), Math.toRadians(180))

                        .splineToConstantHeading(new Vector2d(-36, 60* autoZoneColor.yMult), Math.toRadians(180))
                        .afterTime(0.5,
                                new SequentialAction(
                                        robot.flippers.flipperAction(Flippers.FlipperState.OPEN),
                                        robot.driveClass.updateHeadingFromIMU(autoZoneColor == RED ? -90 : 90)) //TODO: time to be saved
                        )
                        .splineToConstantHeading(new Vector2d(-59.5, 18* autoZoneColor.yMult), Math.toRadians(180))
                        .stopAndAdd(new SequentialAction(
                                robot.intake.intakeAction(1),
                                robot.flippers.flipperAction(Flippers.FlipperState.CLOSED),
                                robot.doubleIntakeAction(),
                                new SleepAction(0.5),
                                robot.lights.lightState(Lights.LightStates.PICKUP),
                                robot.driveClass.updateHeadingFromIMU(autoZoneColor == RED ? -90 : 90)



                        ));
                ;
                break;
            default:
                builder = builder
                        .splineToConstantHeading(new Vector2d(36, 11* autoZoneColor.yMult), Math.toRadians(180))
                        .stopAndAdd( new SequentialAction(robot.pixelMoverAction(Lift.LiftState.DOWN, INTAKE), robot.driveClass.updateHeadingFromIMU(autoZoneColor == RED ? -90 : 90),new SleepAction(1)))
                        .splineToConstantHeading(new Vector2d(12, 11* autoZoneColor.yMult), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-36, 11* autoZoneColor.yMult), Math.toRadians(180))
                        .afterTime(0.25,
                                new SequentialAction(
                                        robot.flippers.flipperAction(Flippers.FlipperState.OPEN))
                        )
                        .splineToConstantHeading(new Vector2d(-46, (autoZoneColor == RED ? 23: 19)* autoZoneColor.yMult), Math.toRadians(180), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-30, 30))
                        .stopAndAdd(robot.driveClass.updateHeadingFromIMU(autoZoneColor == RED ? -90 : 90)) //TODO: time to be saved

                        .strafeToConstantHeading(new Vector2d(-58.5, (autoZoneColor == RED ? 23: 19) * autoZoneColor.yMult), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-30, 30))
                        .stopAndAdd(new SequentialAction(
                                robot.intake.intakeAction(1),
                                robot.flippers.flipperAction(Flippers.FlipperState.CLOSED),
                                robot.doubleIntakeAction(),
                                new SleepAction(0.5),
                                robot.lights.lightState(Lights.LightStates.PICKUP),
                                robot.driveClass.updateHeadingFromIMU(autoZoneColor == RED ? -90 : 90)



                        ));
                break;
        }

        return this;
    }

    public FieldActionSequence toBackDrop(int cycle) {
        switch (autoRoute) {
            case LOOPFAR:
                builder = builder
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(-50, 60* autoZoneColor.yMult), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(12, 60* autoZoneColor.yMult), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(36, 60* autoZoneColor.yMult), Math.toRadians(0))
                        .stopAndAdd(new SequentialAction(robot.intake.intakeAction(0), robot.pixelMoverAction(Lift.LiftState.UP, DROP)))

                        //   .stopAndAdd(new SequentialAction(robot.intake.intakeAction(0), robot.pixelMoverAction(Lift.LiftState.UP, DROP)))
                        .stopAndAdd(new SleepAction(.5))
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(49.5, (32 )* autoZoneColor.yMult), Math.toRadians(8))
                        .stopAndAdd(new SequentialAction(
                                new SleepAction(0.5),
                                robot.claw.clawAction(Claw.ClawPos.OPEN, Claw.Claws.BOTH),
                                robot.lights.lightState(Lights.LightStates.CYCLE)
                        ));
                break;

            case THRU:
            case LOOPMID:
                builder = builder
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(-50, 36* autoZoneColor.yMult), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(12, 36* autoZoneColor.yMult), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(36, 36* autoZoneColor.yMult), Math.toRadians(0))
                        .stopAndAdd(new SequentialAction(robot.intake.intakeAction(0), robot.pixelMoverAction(Lift.LiftState.UP, DROP)))

                        //   .stopAndAdd(new SequentialAction(robot.intake.intakeAction(0), robot.pixelMoverAction(Lift.LiftState.UP, DROP)))
                        .stopAndAdd(new SleepAction(.5))
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(49.5, (36 )* autoZoneColor.yMult), Math.toRadians(8))
                        .stopAndAdd(new SequentialAction(
                                new SleepAction(0.5),
                                robot.claw.clawAction(Claw.ClawPos.OPEN, Claw.Claws.BOTH),
                                robot.lights.lightState(Lights.LightStates.CYCLE)
                        ));
                break;
            default:
                builder = builder
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(-50, 8* autoZoneColor.yMult), Math.toRadians(0))
                        .stopAndAdd(robot.intake.intakeAction(0))
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(12, 8* autoZoneColor.yMult), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(37, 8* autoZoneColor.yMult), Math.toRadians(0))
                        .stopAndAdd(new SequentialAction(robot.intake.intakeAction(0), robot.pixelMoverAction(Lift.LiftState.UP, DROP)))
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(50, (38 )* autoZoneColor.yMult), Math.toRadians(8))
                        .stopAndAdd(new SequentialAction(
                                new SleepAction(0.5),
                                robot.claw.clawAction(Claw.ClawPos.OPEN, Claw.Claws.BOTH),
                                robot.lights.lightState(Lights.LightStates.CYCLE)
                        ));
                break;
        }

        return this;
    }

    public Action build() {
        return builder.build();
    }
}
