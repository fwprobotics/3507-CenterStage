package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class TeleopActionRunner {

    ArrayList<Action> actions = new ArrayList();
    public TeleopActionRunner() {

    }

    public void addAction(Action action) {
        actions.add(action);
    }

    public void update() {
        TelemetryPacket telemetryPacket = new TelemetryPacket();

        actions.forEach((action) -> {
            if (! action.run(telemetryPacket)) {
                actions.remove(action);
            }

        });
    }
}
