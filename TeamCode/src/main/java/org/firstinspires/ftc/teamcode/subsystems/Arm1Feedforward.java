package org.firstinspires.ftc.teamcode.subsystems;

import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.Feedforward;

public class Arm1Feedforward implements Feedforward {
    private double f1;
    private final double ticks_in_degree_1 = 41.8211111111;
    private final double ticks_in_degree_2 = 145.1*28/360; // = 11.2855555556
    //length, COM, mass values for feedforward calculation
    private final double L1 = 43.2;
    private final double L2 = 43.2;
    private final double x1 = 36.96;
    private final double x2 = 26.4;
    private final double m1 = 810;
    private final double m2 = 99.79;
    public Arm1Feedforward(double f){
        f1 = f;
    }

    /**
     * feedforward compute
     * @param v
     * @return ff1(0-1)
     */
    @Override
    public double compute(double v) {
        double target1 = v/ticks_in_degree_1;
        double target2 = ARM2.INSTANCE.motor.getMotor().getTargetPosition()/ticks_in_degree_2;
        double ff1 = (m1*Math.cos(Math.toRadians(target1))*x1 +
        /*NEW*/        m2*Math.cos(Math.atan(((x2*Math.sin(Math.toRadians(target1+target2)))+(L1*Math.sin(Math.toRadians(target1))))/((L1*Math.cos(Math.toRadians(target1)))+(x2*Math.cos(Math.toRadians(target1+target2))))))*
        Math.sqrt(Math.pow((x2*Math.sin(Math.toRadians(target1+target2))+L1*Math.sin(Math.toRadians(target1))),2)+Math.pow((x2*Math.cos(Math.toRadians(target1+target2))+L1*Math.cos(Math.toRadians(target1))),2))) * f1; // feedforward calculation
        return ff1;
    }
}
