package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BLUE - 12 Ball (No Gate)", group = "Blue Close")
public class BLUE_12_NoGate extends BLUE_9_Gate {
    @Override
    protected Mode getMode() {
        return Mode.TWELVE_NO_GATE;
    }
}
