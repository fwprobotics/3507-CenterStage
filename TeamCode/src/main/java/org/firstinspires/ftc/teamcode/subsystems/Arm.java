package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm {

    public static double intake = 1;
    public static double wristIntake = 0.19;//.5;
    public static double drive = 0.5;
    public static  double limbo = 0.8;
    public static double drop = 0.365;//.3;//0.25;
    public static double wristDrop = .37;
    @Config
    static class WristConfig {
        public static  double p = 0.01;
       public static  double i = 0;
        public static double d = 0;
    }

    public enum ArmState {
        INTAKE (intake, wristIntake),
        LIMBO (limbo, 0),

        DRIVE(drive, 0),
        DROP (drop, wristDrop);

        public double armPos;
        public double wristPos;

        ArmState(double armPos, double wristPos) {

            this.armPos = armPos;
            this.wristPos = wristPos;
        }
    }

    public boolean isbusy () {
       boolean isbusy =  (servo.getPosition() >= .75 ||  servo.getPosition() <= .3);
       return  isbusy;
    }
    public ArmState currentState =ArmState.INTAKE;
    public ArmState currentWristState = ArmState.INTAKE;

    ServoImplEx servo;

    Servo wristServo;
    Telemetry telemetry;





    AnalogInput analogInput;

    double lastWristPos;
    double wristRotations = 0;

    PIDController wristPIDcontroller = new PIDController(WristConfig.p, WristConfig.i, WristConfig.d);

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        servo = hardwareMap.get(ServoImplEx.class, "arm");
        wristServo = hardwareMap.get(Servo.class, "wrist");
       // wristServo.setDirection(DcMotorSimple.Direction.REVERSE);
         analogInput = hardwareMap.get(AnalogInput.class, "wristPos");
        this.telemetry = telemetry;
        lastWristPos = getWristAngle(getRawWristAngle());
        if (lastWristPos > 300) {
            wristRotations = -1;
        }
                setState(ArmState.INTAKE);
        setWristSetPoint(ArmState.INTAKE);

    }

    public void setState(ArmState state) {
        this.currentState = state;
        servo.setPosition(state.armPos);
    }

//    public void setWristState(ArmState state) {
//        this.currentWristState = state;
//       // wristServo.setPosition(state.wristPos);
//        wristServo.setPower(state == ArmState.INTAKE? -1 : 11);
//        telemetry.addData("wristPower", (getRawWristAngle() >320|| getRawWristAngle() < 10)? -1 : 1);
//        while (Math.abs(getRawWristAngle()-state.wristPos) > 5) {
//            telemetry.addData("wristAngle", getRawWristAngle());
//            telemetry.update();
//        }
//        wristServo.setPower(0);
//    }

    public void setWristSetPoint(ArmState state) {
        wristServo.setPosition(state.wristPos);
    }

    public void updateWrist() {
//        double wristPos = getRawWristAngle();
//        if (wristPos-lastWristPos < -230) {
//            wristRotations++;
//        } else if (wristPos-lastWristPos > 320) {
//            wristRotations--;
//        }
//        double power = wristPIDcontroller.calculate(getWristAngle(wristPos));
//        wristServo.setPower(power);
//        telemetry.addData("wrist power", power);
//        lastWristPos = wristPos;
//
//        telemetry.addData("calculated wrist angle", getWristAngle(wristPos));
//        telemetry.addData("wrist angle", getRawWristAngle());
//        telemetry.addData("rotation", wristRotations);
    }

    public void moveWrist(double stick) {
       // wristServo.setPower(-stick);
        telemetry.addData("wrist angle", getRawWristAngle());
        telemetry.update();
    }

    public void adjustWristRotation(double adjust) {
        wristRotations += adjust;
    }

    public double getWristAngle(double rawAngle) {


// get the voltage of our analog line
// divide by 3.3 (the max voltage) to get a value between 0 and 1
// multiply by 360 to convert it to 0 to 360 degrees
        double position = rawAngle + (wristRotations*360);
        return position;
    }

    public double getRawWristAngle() {


// get the voltage of our analog line
// divide by 3.3 (the max voltage) to get a value between 0 and 1
// multiply by 360 to convert it to 0 to 360 degrees
        double position = analogInput.getVoltage() / 3.3 * 360;
        return position;
    }


    public class SetArmState implements Action {

        ArmState state;

        public SetArmState(ArmState state) {
            this.state = state;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setState(this.state);
            return false;
        }
    }

    public Action armStateAction(ArmState state) {
        return new SetArmState(state);
    }
    public Action wristStateAction(ArmState state) {
        return telemetryPacket -> {
            setWristSetPoint(state);
            return false;
        };
    }

    public Action wristUpdateAction() {
        return telemetryPacket -> {
            updateWrist();
            return true;
        };
    }


}
