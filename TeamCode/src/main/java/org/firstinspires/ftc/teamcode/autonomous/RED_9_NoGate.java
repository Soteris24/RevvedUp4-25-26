package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RED - 9 Ball (No Gate)", group = "Red Close")
public class RED_9_NoGate extends RED_9_Gate {
    @Override
    protected Mode getMode() {
        return Mode.NINE_NO_GATE;
    }
}
