package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(this, hardwareMap, telemetry);
        Carousel carousel = new Carousel(hardwareMap, telemetry);
      //  Arm arm = new Arm(hardwareMap, telemetry);

        waitForStart();
        if (isStopRequested()) return;
        while(opModeIsActive()) {
            drivetrain.JoystickMovement(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_bumper, false, gamepad2.right_bumper);
            if (gamepad2.a) carousel.nextState(true);
            else if (gamepad2.b) carousel.setState(Carousel.CarouselStates.SLOT1, true);
            else if (gamepad2.x) carousel.setState(Carousel.CarouselStates.SLOT2, true);
            else if (gamepad2.y) carousel.dropBoth();
        }
    }
}
