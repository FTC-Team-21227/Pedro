package org.firstinspires.ftc.teamcode.subsystems;

import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.Feedforward;

public class Arm2Feedforward implements Feedforward {
    private double f2;
    private final double ticks_in_degree_1 = TunePID_MotionProfile.ticks_in_degree_1;
    private final double ticks_in_degree_2 = TunePID_MotionProfile.ticks_in_degree_2;
    //length, COM, mass values for feedforward calculation
    private final double L1 = TunePID_MotionProfile.L1;
    private final double L2 = TunePID_MotionProfile.L2;
    private final double x1 = TunePID_MotionProfile.x1;
    private final double x2 = TunePID_MotionProfile.x2;
    private final double m1 = TunePID_MotionProfile.m1;
    private final double m2 = TunePID_MotionProfile.m2;
    private final double ARM1_OFFSET = TunePID_MotionProfile.ARM1_OFFSET;
    private final double ARM2_OFFSET = TunePID_MotionProfile.ARM2_OFFSET;
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
        double target1 = ARM1.INSTANCE.motor.getMotor().getTargetPosition()/ticks_in_degree_1;
        double theta1_actual = Math.toRadians(target1 + ARM1_OFFSET);
        double theta2_actual = Math.toRadians(target1 + ARM1_OFFSET + target2 + ARM2_OFFSET);
        double ff2 = m2 * x2 * Math.cos(theta2_actual) * f2;
        return ff2;
    }
}
