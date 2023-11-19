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

    public boolean isBusy() {
        return actions.size() > 0;
    }

    public void update() {
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        ArrayList<Action> toRemove = new ArrayList<>();
        actions.forEach((action) -> {
            if (! action.run(telemetryPacket)) {
                toRemove.add(action);
            }

        });

        toRemove.forEach((action -> {
            actions.remove(action);
        }));
    }
}
