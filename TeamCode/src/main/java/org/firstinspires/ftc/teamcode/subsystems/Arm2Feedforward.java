package org.firstinspires.ftc.teamcode.subsystems;

import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.Feedforward;

public class Arm2Feedforward implements Feedforward {
    private double f2;
    private final double ticks_in_degree_1 = 41.8211111111;
    private final double ticks_in_degree_2 = 145.1*28/360; // = 11.2855555556
    //length, COM, mass values for feedforward calculation
    private final double L1 = 43.2;
    private final double L2 = 43.2;
    private final double x1 = 36.96;
    private final double x2 = 26.4;
    private final double m1 = 810;
    private final double m2 = 99.79;
    public Arm2Feedforward(double f){
        f2 = f;
    }

    /**
     * feedforward compute
     * @param v
     * @return ff1(0-1)
     */
    @Override
    public double compute(double v) {
        double target2 = v/ticks_in_degree_2;
        double ff2 = (m2*Math.cos(Math.toRadians(ARM1.INSTANCE.motor.getMotor().getTargetPosition()/ticks_in_degree_1+target2))*x2) * f2; // feedforward calculation
        return ff2;
    }
}
