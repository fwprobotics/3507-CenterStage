package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) throws IOException {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17, 17)
                .build();

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(67, -35, Math.toRadians(180)))
//                .lineToX(35)
//                .turn(Math.toRadians(90))
//                        .lineToY(24)
//
//                .splineTo(new Vector2d(30, 50), Math.toRadians(90))
//                .build());
        Robot robot = new Robot(Robot.AutoZoneColor.RED, Robot.AutoZoneHalf.NEAR,myBot);
        myBot.runAction(robot.createFieldActionSequence(new Pose2d(10, -64, Math.toRadians(-90)))
                                                        .dropPurplePixel(Robot.PropLocation.CENTER)
                                                        .dropYellowPixel(Robot.PropLocation.CENTER)
                                .toStack(0)
                                .toBackDrop(0)
                               // .toStack()
                                .park()
                                                        .build());
//        Robot.AutoZoneColor autoZone = Robot.AutoZoneColor.BLUE;
//        Robot.AutoZoneHalf autoZoneHalf = Robot.AutoZoneHalf.TOP;
//
//        Robot.PropLocation propLocation = Robot.PropLocation.LEFT;
//        Action myAction;
//        switch (autoZoneHalf) {
//            case BOTTOM:
//            switch (propLocation) {
//                case LEFT:
//                    myAction =
//                            myBot.getDrive().actionBuilder(new Pose2d(62 * autoZone.xMult, -35, Math.toRadians(180 * autoZone.xMult))).lineToX(35 * autoZone.xMult).turn(Math.toRadians(90)).stopAndAdd(new SleepAction(1)).lineToY(24).splineTo(new Vector2d(30 * autoZone.xMult, 50), Math.toRadians(90)).strafeTo(new Vector2d(62 * autoZone.xMult, 50)).strafeTo(new Vector2d(62 * autoZone.xMult, 62)).build();
//                    break;
//                case CENTER:
//                    myAction =
//                            myBot.getDrive().actionBuilder(new Pose2d(62 * autoZone.xMult, -35, Math.toRadians(180 * autoZone.xMult))).lineToX(35 * autoZone.xMult).stopAndAdd(new SleepAction(1)).turn(Math.toRadians(90)).lineToY(24).splineTo(new Vector2d(35 * autoZone.xMult, 50), Math.toRadians(90)).strafeTo(new Vector2d(62 * autoZone.xMult, 50)).strafeTo(new Vector2d(62 * autoZone.xMult, 62)).build();
//                    break;
//                default:
//                    myAction =
//                            myBot.getDrive().actionBuilder(new Pose2d(62 * autoZone.xMult, -35, Math.toRadians(180 * autoZone.xMult))).lineToX(35 * autoZone.xMult).turn(Math.toRadians(90)).lineToY(-15).stopAndAdd(new SleepAction(1)).lineToY(24).splineTo(new Vector2d(30 * autoZone.xMult, 50), Math.toRadians(90)).strafeTo(new Vector2d(62 * autoZone.xMult, 50)).strafeTo(new Vector2d(62 * autoZone.xMult, 62)).build();
//            }
//            break;
//            default:
//                switch (propLocation) {
//                    case LEFT:
//                        myAction =
//                                myBot.getDrive().actionBuilder(new Pose2d(62 * autoZone.xMult, 11, Math.toRadians(180 * autoZone.xMult))).lineToX(35 * autoZone.xMult).turn(Math.toRadians(90)).stopAndAdd(new SleepAction(1)).strafeTo(new Vector2d(30 * autoZone.xMult, 50)).strafeTo(new Vector2d(62 * autoZone.xMult, 50)).strafeTo(new Vector2d(62 * autoZone.xMult, 62)).build();
//                        break;
//                    case CENTER:
//                        myAction =
//                                myBot.getDrive().actionBuilder(new Pose2d(62 * autoZone.xMult, 11, Math.toRadians(180 * autoZone.xMult))).lineToX(35 * autoZone.xMult).stopAndAdd(new SleepAction(1)).turn(Math.toRadians(90)).strafeTo(new Vector2d(35 * autoZone.xMult, 50)).strafeTo(new Vector2d(62 * autoZone.xMult, 50)).strafeTo(new Vector2d(62 * autoZone.xMult, 62)).build();
//                        break;
//                    default:
//                        myAction =
//                                myBot.getDrive().actionBuilder(new Pose2d(62 * autoZone.xMult, 11, Math.toRadians(180 * autoZone.xMult))).lineToX(35 * autoZone.xMult).turn(Math.toRadians(90)).lineToY(31).stopAndAdd(new SleepAction(1)).strafeTo(new Vector2d(30 * autoZone.xMult, 50)).strafeTo(new Vector2d(62 * autoZone.xMult, 50)).strafeTo(new Vector2d(62 * autoZone.xMult, 62)).build();
//                }
//
//        }
//        myBot.runAction(myAction);
//        File imageFile = new File("MeepMeepTesting/src/main/java/com/example/meepmeeptesting/image.png");
//        Image image = ImageIO.read(imageFile);
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();


    }
}