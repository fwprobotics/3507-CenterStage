package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pipelines.WallProcessor;

public class Claw {

    public enum ClawPos {
        CLOSED (0, 1),
        OPEN (1, 0);

        double leftPos;
        double rightPos;

        ClawPos(double rightPos, double leftPos) {
            this.rightPos = rightPos;
            this.leftPos = leftPos;
        }
    }

    public enum Claws {
        RIGHT,
        LEFT,
        BOTH
    }

    Servo clawRight;
    Servo clawLeft;

    NormalizedColorSensor colorRight;
    NormalizedColorSensor colorLeft;

    WallProcessor.COLORS pixelRight = WallProcessor.COLORS.NONE;
    WallProcessor.COLORS pixelLeft = WallProcessor.COLORS.NONE;

    Telemetry telemetry;

    ClawPos clawState;

    public Claw(HardwareMap hardwareMap, Telemetry telemetry) {
        //define claw servos
        clawRight = hardwareMap.servo.get("clawRight");
        clawLeft = hardwareMap.servo.get("clawLeft");

        //define color sensors
        colorRight = hardwareMap.get(NormalizedColorSensor.class, "clawRight");
        colorLeft = hardwareMap.get(NormalizedColorSensor.class, "clawLeft");

        this.telemetry = telemetry;

        //TODO: set default pos lock

    }

    public void setClawPosition(ClawPos pos, Claws claw) {
        clawState = pos;
        if (claw == Claws.RIGHT || claw == Claws.BOTH) {
            if (pos == ClawPos.OPEN) pixelRight = WallProcessor.COLORS.NONE;
            clawRight.setPosition(pos.rightPos);
        }
        if (claw == Claws.LEFT || claw == Claws.BOTH) {
            if (pos == ClawPos.OPEN) pixelLeft = WallProcessor.COLORS.NONE;
            clawLeft.setPosition(pos.leftPos);
        }
    }

    public boolean isPixel(NormalizedRGBA color) {
        return color.red != 0;
    }

    public Action clawAction(ClawPos pos, Claws claw) {
        return telemetryPacket -> {
            setClawPosition(pos, claw);
            return false;
        };
    }

    public boolean update(Arm.ArmState armState, Lift.LiftState liftState) {
        telemetry.addData("Right Claw Open", clawRight.getPosition() == ClawPos.OPEN.rightPos);
        telemetry.addData("Right Claw Open", clawLeft.getPosition() == ClawPos.OPEN.leftPos);
        if (armState == Arm.ArmState.INTAKE && liftState == Lift.LiftState.DOWN) {
            if (clawLeft.getPosition() == ClawPos.OPEN.leftPos) {
                if (isPixel(colorLeft.getNormalizedColors())) { //replace with something that actually tells if pixel
                    setClawPosition(ClawPos.CLOSED, Claws.LEFT);
                }
            }
            if (clawRight.getPosition() == ClawPos.OPEN.rightPos) {
                if (isPixel(colorRight.getNormalizedColors())) {
                    setClawPosition(ClawPos.CLOSED, Claws.RIGHT);
                }
            }
        }
        return clawRight.getPosition() == ClawPos.CLOSED.rightPos && clawLeft.getPosition() == ClawPos.OPEN.leftPos;
    }
}
