package org.firstinspires.ftc.teamcode.mathUtils;

public class ShooterCalculator {

    double[]  distanceTable;
    double[] rpmTable;

    public ShooterCalculator(double[] distanceTable, double[] rpmTable) {
        this.distanceTable = distanceTable;
        this.rpmTable = rpmTable;
    }


    public double calculateRPM(double distance){

        if (distance == -1)
            return rpmTable[2];
        if (distance <= distanceTable[0])
            return rpmTable[0];
        if (distance >= distanceTable[distanceTable.length - 1])
            return rpmTable[rpmTable.length - 1];

        for(int i = 0; i < distanceTable.length - 1; i++) {
            double x0 = distanceTable[i];
            double x1 = distanceTable[i+1];

            double y0 = rpmTable[i];
            double y1 = rpmTable[i+1];

            if(distance >= x0 && distance <= x1) {
                return y0 + (distance - x0) * (y1 - y0) / (x1 - x0);
            }
        }
        return rpmTable[rpmTable.length - 1];
    }
}
