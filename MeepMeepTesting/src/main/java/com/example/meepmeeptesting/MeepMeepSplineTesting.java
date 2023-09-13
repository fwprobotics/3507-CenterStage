package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

//Chases Vanity Project
public class MeepMeepSplineTesting {
    public static void main(String[] args) throws IOException {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d initialPose = new Pose2d(67, -35, Math.toRadians(180));
        Vector2d p1 = new Vector2d(initialPose.position.x,initialPose.position.y);
        Vector2d p2 = new Vector2d(56,0);
        Vector2d p3 = new Vector2d(30,40);
        Vector2d p4 = new Vector2d(38,0);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        double p1p3tangent = .5 * (Math.PI - Math.atan(p1.y - p3.y / p1.x - p3.x));

        double p2p4tangent = .5 * (Math.PI - Math.atan(p2.y - p4.y / p2.x - p4.x));

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .splineTo(p2,
                        p1p3tangent
                          )
                .splineTo(p3,
                        p2p4tangent

                )
                .splineTo(p4,Math.toRadians(270))
                .splineTo(new Vector2d(38,-55),Math.toRadians(270))
                .splineTo(new Vector2d( p1.x-10,p1.y),

                        p1p3tangent

                )
                .build());

        File imageFile = new File("/Users/chasewayland/3507-CenterStage/MeepMeepTesting/src/main/java/com/example/meepmeeptesting/image.png");
        Image image = ImageIO.read(imageFile);
        meepMeep.setBackground(image)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();


    }
}