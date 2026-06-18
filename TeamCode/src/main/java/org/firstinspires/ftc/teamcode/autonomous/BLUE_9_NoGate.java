package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BLUE - 9 Ball (No Gate)", group = "Blue Close")
public class BLUE_9_NoGate extends BLUE_9_Gate {
    @Override
    protected Mode getMode() {
        return Mode.NINE_NO_GATE;
    }
}
