package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    public static double intakeSpeed = 0.9;

    DcMotor intakeMotor;
    Telemetry telemetry;
    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
    }

    public void manualControl(double rightTrigger, double leftTrigger) {
        if (rightTrigger > 0 ) {
            intakeMotor.setPower(rightTrigger * intakeSpeed);
        } else {
            intakeMotor.setPower(-leftTrigger*intakeSpeed);
        }
    }

    public void setState(boolean on) {
        intakeMotor.setPower(intakeSpeed);
    }
}
