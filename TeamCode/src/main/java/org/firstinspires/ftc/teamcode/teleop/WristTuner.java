package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

@TeleOp
public class WristTuner extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()) {
            arm.moveWrist(gamepad2.left_stick_y);
        }
    }
}
