package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RED - 12 Ball (No Gate)", group = "Red Close")
public class RED_12_NoGate extends RED_9_Gate {
    @Override
    protected Mode getMode() {
        return Mode.TWELVE_NO_GATE;
    }
}
