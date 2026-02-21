package org.firstinspires.ftc.teamcode.tests;

import android.graphics.Color;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class colorSensorTest2 {

    RobotHardware hw;
    public colorSensorTest2(RobotHardware hw){
        this.hw = hw;
    }
    void update(){
        int r = hw.colorSensor.red();
        int g = hw.colorSensor.green();
        int b = hw.colorSensor.blue();
        int a = hw.colorSensor.alpha();

        if (a == 0) a=1;

        float rNorm = (float) r / a;
        float gNorm = (float) g / a;
        float bNorm = (float) b / a;

        int rScaled = (int) (rNorm * 255);
        int gScaled = (int) (gNorm * 255);
        int bScaled = (int) (bNorm * 255);

        rScaled = Math.min(255, Math.max(0, rScaled));
        gScaled = Math.min(255, Math.max(0, gScaled));
        bScaled = Math.min(255, Math.max(0, bScaled));

        float[] hsv = new float[3];
        Color.RGBToHSV(rScaled, gScaled, bScaled, hsv);

        float hue = hsv[0];
        float saturation = hsv[1];
        float value = hsv[2];
    }

}
