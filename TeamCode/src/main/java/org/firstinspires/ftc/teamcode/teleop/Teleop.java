package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.AutoPark.WALL;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Airplane;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Climb;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Flippers;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.ToggleButton;
import org.firstinspires.ftc.teamcode.subsystems.Wacker;
import org.firstinspires.ftc.teamcode.util.TeleopActionRunner;

@TeleOp
public class Teleop extends LinearOpMode {
    boolean fieldrelative = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(this, hardwareMap, telemetry);
        Claw claw = new Claw(hardwareMap, telemetry);
     //   ToggleButton carouselState = new ToggleButton(false);
        Lift lift = new Lift(hardwareMap, telemetry);
        GamepadEx gamepadex1 = new GamepadEx(gamepad1);
        GamepadEx gamepadex2 = new GamepadEx(gamepad2);
       // carouselState.toggle(gamepad2.a);
        Arm arm = new Arm(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap, telemetry);
        Airplane airplane = new Airplane(hardwareMap, telemetry);
        Climb climb = new Climb(hardwareMap, telemetry);
        Flippers flippers = new Flippers(hardwareMap, telemetry);
        Lights lights = new Lights(hardwareMap, telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Robot robot = new Robot(Robot.AutoZoneColor.RED, Robot.AutoZoneHalf.FAR, Robot.AutoRoute.DEFAULT, WALL, drive, arm, claw, lift, intake, flippers, lights, null);
        TeleopActionRunner actionRunner = new TeleopActionRunner();
        ElapsedTime time = new ElapsedTime();
        ToggleButton yToggle = new ToggleButton(false);
        telemetry.addData("Field Relative", fieldrelative);
        claw.setClawPosition(Claw.ClawPos.OPEN, Claw.Claws.BOTH);
        waitForStart();
        lights.setState(Lights.LightStates.DEFAULT);
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
//            if (gamepad1.right_bumper) {
//                climb.setHookState(Climb.HookStates.UP);
//            }
//            else if (gamepad1.left_bumper) {
//                climb.setHookState(Climb.HookStates.DOWN);
//            }
            if (gamepadex1.wasJustPressed(GamepadKeys.Button.A) && !climb.active) {
                climb.setWinchState(Climb.WinchStates.UP);
            }
            else if (gamepadex1.wasJustPressed(GamepadKeys.Button.B) && !climb.active) {
                climb.setWinchState(Climb.WinchStates.DOWN);
            }
            else if ((gamepadex1.wasJustPressed(GamepadKeys.Button.A) || gamepadex1.wasJustPressed(GamepadKeys.Button.B) ) && climb.active){
                climb.setWinchState(Climb.WinchStates.OFF);
            }
            if (fieldrelative) {
                if (gamepad1.x) {
                    drivetrain.imu.resetYaw();
                }
            }
            yToggle.toggle(true);
            //Operator controls
            if (gamepadex1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) || gamepadex2.wasJustPressed(GamepadKeys.Button.X)) {
                if (claw.stateLeft == Claw.ClawPos.OPEN) {
                    claw.setClawPosition(Claw.ClawPos.CLOSED, Claw.Claws.LEFT);
                } else {
                    claw.setClawPosition(Claw.ClawPos.OPEN, Claw.Claws.LEFT);
                }
                lights.displayClawState(claw);
            }

            else if (gamepadex1.wasJustPressed(GamepadKeys.Button.X)) {

                lift.resetEncoders();

            }
            else if (gamepadex1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT) || gamepadex2.wasJustPressed(GamepadKeys.Button.B)) {
                //toggle right
                if (claw.stateRight == Claw.ClawPos.OPEN) {
                    claw.setClawPosition(Claw.ClawPos.CLOSED, Claw.Claws.RIGHT);
                } else {
                    claw.setClawPosition(Claw.ClawPos.OPEN, Claw.Claws.RIGHT);
                }

                lights.displayClawState(claw);
            //    claw.setClawPosition(Claw.ClawPos.OPEN, Claw.Claws.BOTH);
            }
            else if (gamepadex1.wasJustPressed(GamepadKeys.Button.DPAD_UP) || gamepadex2.wasJustPressed(GamepadKeys.Button.Y)){
                //toggle both
                checkClawState(claw, arm, lights);
            }
           // lift.manualControl(-gamepad2.right_stick_y);
            if (gamepad2.right_bumper && (lift.liftMotor.getCurrentPosition() > Lift.LiftState.LOW.height)){
                arm.setState(Arm.ArmState.INTAKE);
                arm.setWristSetPoint(Arm.ArmState.INTAKE);
               // arm.setWristState(Arm.ArmState.INTAKE);
            }
            else if (gamepad2.left_bumper && (lift.liftMotor.getCurrentPosition() > Lift.LiftState.LOW.height)){
                arm.setState(Arm.ArmState.DROP);
                arm.setWristSetPoint(Arm.ArmState.DROP);
              //  arm.setWristState(Arm.ArmState.DROP);
            }

            if (gamepad2.touchpad && !actionRunner.isBusy()) {
                actionRunner.addAction(robot.doubleIntakeAction());
            }

            if (gamepad2.start) {
                flippers.setFlipperPosition(Flippers.FlipperState.CLOSED);
            } else if (gamepad2.back) {
                flippers.setFlipperPosition(Flippers.FlipperState.OPEN);
            }

            //lift arm
            if (!actionRunner.isBusy()) {
                if (gamepadex2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    actionRunner.addAction(robot.pixelMoverAction(Lift.LiftState.DOWN, Arm.ArmState.INTAKE));
                } else if (gamepadex2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    actionRunner.addAction(robot.pixelMoverAction(Lift.LiftState.MID, Arm.ArmState.DROP));
                } else if (gamepadex2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    actionRunner.addAction(robot.pixelMoverAction(Lift.LiftState.UP, Arm.ArmState.DROP));
                } else if (gamepadex2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    actionRunner.addAction(robot.pixelMoverAction(Lift.LiftState.UPPER, Arm.ArmState.DROP));
                }
            }


        //    arm.moveWrist(gamepad2.left_stick_y);



            //red for 1, green for 2, blue for lock
//            gamepad2.setLedColor(carousel.currentState == Carousel.CarouselStates.SLOT1 ? 255 :0,
//                    carousel.currentState == Carousel.CarouselStates.SLOT2 ? 255 : 0,
//                    carousel.currentState == Carousel.CarouselStates.LOCK ? 255 : 0,
//                    100);
            if (Math.floor(time.seconds()) == 90) {
                gamepad1.rumble(3000);
            }

            if (!gamepad2.a) {
                claw.update(arm.currentState, Lift.LiftState.DOWN, lights);
            }
            intake.manualControl(gamepad2.left_trigger, gamepad2.right_trigger);
            lift.manualControl(-gamepad2.right_stick_y, gamepadex2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON), gamepadex2.wasJustReleased(GamepadKeys.Button.RIGHT_STICK_BUTTON));
            arm.updateWrist();
            actionRunner.update();
            gamepadex1.readButtons();
            gamepadex2.readButtons();

        //    if (!arm.isbusy()) {
          //  lift.teleOpControl(gamepad2);




            telemetry.addData("slow mode on", (gamepad1.right_trigger >= .1));
            telemetry.addData("Boost mode on", (gamepad1.left_trigger >= .1));
            telemetry.addData("arm state", arm.currentState);
            telemetry.update();
        }
    }

    public void checkClawState(Claw claw, Arm arm, Lights lights) {
        if (claw.stateLeft != claw.stateRight) {
                claw.setClawPosition(Claw.ClawPos.OPEN, Claw.Claws.BOTH);
            return;
        }

        if (claw.stateLeft == Claw.ClawPos.OPEN)
        {
            claw.setClawPosition(Claw.ClawPos.CLOSED, Claw.Claws.BOTH);
        }
        else
        {
            claw.setClawPosition(Claw.ClawPos.OPEN, Claw.Claws.BOTH);
        }

        lights.displayClawState(claw);

}}
