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
        CLOSED (0.75, 0.25),
        OPEN (0, 1);

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

    int colorRightPrev = 0;
    int colorLeftPrev = 0;

    public ClawPos stateRight;
    public ClawPos stateLeft;

    WallProcessor.COLORS pixelRight = WallProcessor.COLORS.NONE;
    WallProcessor.COLORS pixelLeft = WallProcessor.COLORS.NONE;

    Telemetry telemetry;

    ClawPos clawState;

    public Claw(HardwareMap hardwareMap, Telemetry telemetry) {
        //define claw servos
        clawRight = hardwareMap.servo.get("clawRight");
        clawLeft = hardwareMap.servo.get("clawLeft");

        //define color sensors
        colorRight = hardwareMap.get(NormalizedColorSensor.class, "colorRight");
        colorLeft = hardwareMap.get(NormalizedColorSensor.class, "colorLeft");

        this.telemetry = telemetry;

        //TODO: set default pos lock

    }

    public void setClawPosition(ClawPos pos, Claws claw) {
        clawState = pos;
        if (claw == Claws.RIGHT || claw == Claws.BOTH) {
            stateRight = pos;
            if (pos == ClawPos.OPEN) pixelRight = WallProcessor.COLORS.NONE;
            clawRight.setPosition(pos.rightPos);
        }
        if (claw == Claws.LEFT || claw == Claws.BOTH) {
            stateLeft = pos;
            if (pos == ClawPos.OPEN) pixelLeft = WallProcessor.COLORS.NONE;
            clawLeft.setPosition(pos.leftPos);
        }
    }

    public boolean isPixel(NormalizedRGBA color, boolean right) {
        if (!right) {
            if (color.toColor() - colorLeftPrev <= 10000000) return false;
        } else {
            if (color.toColor() - colorRightPrev <= 10000000) return false;
        }
        boolean white = color.toColor()-  (-15912313) <= 10000000;
        boolean yellow =  color.toColor()-  (-268303610) <= 10000000;
        boolean purple =  color.toColor()-  (-49671931) <= 10000000;
        boolean green = color.toColor()-  (-66843130) <= 10000000;

        if (white) {
            if (!right) {
                pixelLeft = WallProcessor.COLORS.WHITE;
            } else {
                pixelRight = WallProcessor.COLORS.WHITE;
            }
        } else if (yellow) {
        if (!right) {
            pixelLeft = WallProcessor.COLORS.YELLOW;
        } else {
            pixelRight = WallProcessor.COLORS.YELLOW;
        }
        } else if (purple) {
            if (!right) {
                pixelLeft = WallProcessor.COLORS.PURPLE;
            } else {
                pixelRight = WallProcessor.COLORS.PURPLE;
            }
        } else if (green) {
            if (!right) {
                pixelLeft = WallProcessor.COLORS.GREEN;
            } else {
                pixelRight = WallProcessor.COLORS.GREEN;
            }
        }
        if (!right) colorLeftPrev = color.toColor();
        else colorRightPrev = color.toColor();
        return (white || yellow || purple || green);
        //This worked but doesn't specify colors
       // return color.toColor()-  (-15912313) <= 50000000 || color.toColor()-  (-268303610) <= 50000000 || color.toColor()-  (-49671931) <= 50000000 || color.toColor()-  (-66843130) <= 50000000;
    }

    public Action clawAction(ClawPos pos, Claws claw) {
        return telemetryPacket -> {
            setClawPosition(pos, claw);
            return false;
        };
    }

    public boolean update(Arm.ArmState armState, Lift.LiftState liftState) {
        telemetry.addData("Right Claw Open", clawRight.getPosition() == ClawPos.OPEN.rightPos);
        telemetry.addData("Left Claw Open", clawLeft.getPosition() == ClawPos.OPEN.leftPos);
        telemetry.addData("Right Claw Color", colorRight.getNormalizedColors().toColor());
        telemetry.addData("Left Claw Color", colorLeft.getNormalizedColors().toColor());
        telemetry.addData("Right Claw Pixel", pixelRight);
        telemetry.addData("Left Claw Pixel", pixelLeft);
        if (armState == Arm.ArmState.INTAKE && liftState == Lift.LiftState.DOWN) {
            if (clawLeft.getPosition() == ClawPos.OPEN.leftPos) {
                if (isPixel(colorLeft.getNormalizedColors(), false)) { //replace with something that actually tells if pixel
                    setClawPosition(ClawPos.CLOSED, Claws.LEFT);
                }
            }
            if (clawRight.getPosition() == ClawPos.OPEN.rightPos) {
                if (isPixel(colorRight.getNormalizedColors(), true)) {
                    setClawPosition(ClawPos.CLOSED, Claws.RIGHT);
                }
            }
        }
        return clawRight.getPosition() == ClawPos.CLOSED.rightPos && clawLeft.getPosition() == ClawPos.CLOSED.leftPos;
    }
}
