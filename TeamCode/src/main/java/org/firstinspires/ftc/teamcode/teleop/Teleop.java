package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Airplane;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.Climb;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ToggleButton;

@TeleOp
public class Teleop extends LinearOpMode {
    boolean fieldrelative = false;
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
        Airplane airplane = new Airplane(hardwareMap, telemetry);
        Climb climb = new Climb(hardwareMap, telemetry);
        ElapsedTime time = new ElapsedTime();
        telemetry.addData("Field Relative", fieldrelative);
        waitForStart();
        if (isStopRequested()) return;
        while(opModeIsActive()) {
            drivetrain.JoystickMovement(gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    gamepad1.right_stick_y,
                    (gamepad1.right_trigger >= .1),
                    fieldrelative,
                    (gamepad1.left_trigger >= .1));
            // Driver controls

            if (gamepad1.y) {
                airplane.setAirplaneState(Airplane.AirplaneStates.FIRE);
            }
            if (gamepad1.right_bumper) {
                climb.setHookState(Climb.HookStates.UP);
            }
            else if (gamepad1.left_bumper) {
                climb.setHookState(Climb.HookStates.DOWN);
            }
            if (gamepad1.a) {
                climb.setWinchPower(Climb.ClimbConfig.winchSpeed);
            }
            else if (gamepad1.b) {
                climb.setWinchPower(-Climb.ClimbConfig.winchSpeed);
            }
            else {
                climb.setWinchPower(0);
            }
            if (fieldrelative) {
                if (gamepad1.x) {
                    drivetrain.imu.resetYaw();
                }
            }

            //Operator controls
            if (gamepad2.x && !prevGamepad.x) {
                carousel.nextState(true);
            }
            else if (gamepad2.a) {
                carousel.setState(Carousel.CarouselStates.SLOT1, arm.currentState== Arm.ArmState.INTAKE);
            }
            else if (gamepad2.b) {
                carousel.setState(Carousel.CarouselStates.SLOT2, arm.currentState== Arm.ArmState.INTAKE);
            }
            else if (gamepad2.y){
                carousel.setState(Carousel.CarouselStates.LOCK, arm.currentState== Arm.ArmState.INTAKE);
            }
           // lift.manualControl(-gamepad2.right_stick_y);
            if (gamepad2.right_bumper){
                arm.setState(Arm.ArmState.INTAKE);
                carousel.setState(Carousel.CarouselStates.SLOT1,true);
            }
            else if (gamepad2.left_bumper){
                arm.setState(Arm.ArmState.DROP);
            }
            //red for 1, green for 2, blue for lock
            gamepad2.setLedColor(carousel.currentState == Carousel.CarouselStates.SLOT1 ? 255 :0,
                    carousel.currentState == Carousel.CarouselStates.SLOT2 ? 255 : 0,
                    carousel.currentState == Carousel.CarouselStates.LOCK ? 255 : 0,
                    100);
            if (time.seconds() == 90) {
                gamepad1.rumble(3000);
            }

            intake.manualControl(gamepad2.left_trigger, gamepad2.right_trigger);
        //    if (!arm.isbusy()) {
                lift.teleOpControl(gamepad2);
        //    }
            prevGamepad.copy(gamepad2);


            telemetry.addData("slow mode on", (gamepad1.right_trigger >= .1));
            telemetry.addData("Boost mode on", (gamepad1.left_trigger >= .1));
            telemetry.addData("arm state", arm.currentState);
            telemetry.update();
        }
    }
}
