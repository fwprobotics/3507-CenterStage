package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ToggleButton;

@TeleOp
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(this, hardwareMap, telemetry);
        Carousel carousel = new Carousel(hardwareMap, telemetry);
     //   ToggleButton carouselState = new ToggleButton(false);
        Lift lift = new Lift(hardwareMap, telemetry);
        Gamepad prevGamepad = new Gamepad();
        prevGamepad.copy(gamepad2);
       // carouselState.toggle(gamepad2.a);
        Arm arm = new Arm(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap, telemetry);

        waitForStart();
        if (isStopRequested()) return;
        while(opModeIsActive()) {
            drivetrain.JoystickMovement(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_bumper, false, gamepad2.right_bumper);
            if (gamepad2.a) carousel.nextState(true);
            else if (gamepad2.b) carousel.setState(Carousel.CarouselStates.SLOT1, arm.currentState== Arm.ArmState.INTAKE);
            else if (gamepad2.x) carousel.setState(Carousel.CarouselStates.SLOT2, arm.currentState== Arm.ArmState.INTAKE);
            else if (gamepad2.y) carousel.setState(Carousel.CarouselStates.LOCK, arm.currentState== Arm.ArmState.INTAKE);
            lift.manualControl(-gamepad2.right_stick_y);
            if (gamepad2.right_bumper) arm.setState(Arm.ArmState.DROP);
            else if (gamepad2.left_bumper) arm.setState(Arm.ArmState.INTAKE);
            if (gamepad2.dpad_down) lift.setHeight(Lift.LiftState.UP.height);
            intake.manualControl(gamepad2.right_trigger, gamepad2.left_trigger);
            lift.update();
            prevGamepad.copy(gamepad2);
            telemetry.addData("arm state", arm.currentState);
        }
    }
}
