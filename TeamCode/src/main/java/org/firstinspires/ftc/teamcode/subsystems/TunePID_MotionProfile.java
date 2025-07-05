package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class TunePID_MotionProfile extends OpMode {
    private DcMotorEx ARM1; //bottom arm
    private DcMotorEx ARM2; //top arm
    //PID controllers for ARM1 and ARM2
    private PIDController controller1;
    private PIDController controller2;
    //PIDF gains
    public static double p1 = 0.008, i1 = 0, d1 = 0.0002;
    public static double f1 = 0.000001;
    public static double p2 = 0.006, i2 = 0, d2 = 0.0001;
    public static double f2 = 0.000006;
    //ARM1, ARM2 target positions, in degrees
    double target1 = 0;
    double target2 = 0;
    double Target1 = 0;
    double Target2 = 0;
    public static double userTarget1 = 0;
    public static double userTarget2 = 0;
    //ticks to degrees conversion, very useful
    public final static double ticks_in_degree_1 = 384.539792388 * 28.0/360; // = 29.9086505191 //537.7*28/360; // = 41.8211111111
    public final static double ticks_in_degree_2 = 103.764705882 * 28.0/360; // = 8.07058823527 //145.1*28/360; // = 11.2855555556
    public static final double L1 = 43.2;
    public static final double L2 = 43.2;
    public static final double x1 = 36.96;
    public static final double x2 = 26.4; //31.48;
    public static final double m1 = 810;
    public static final double m2 = 404.273; //99.79;
    // ARM1 Motion Profile Variables
    private double arm1StartPos;
    private double arm1TargetPos;
    private double arm1DDec;
    private boolean arm1Decelerating = false;

    // ARM2 Motion Profile Variables
    private double arm2StartPos;
    private double arm2TargetPos;
    private double arm2DDec;
    private boolean arm2Decelerating = false;
    public static final double ARM1_OFFSET = -17.039;
    public static final double ARM2_OFFSET = 17.443;

    // Profile Parameters (tune these!)
    public static double V_MAX = 200000.0; // Encoder ticks/sec (≈ 100° /sec if 1 tick/degree)
    public static double A_DEC = 70000.0; // Ticks/sec² (adjust for smooth stopping)
    public static double LOOP_TIME = 0.02; // 20ms (typical FTC loop time)
    @Override
    public void init(){
        controller1 = new PIDController(p1,i1,d1);
        controller2 = new PIDController(p2,i2,d2);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        ARM1 = hardwareMap.get(DcMotorEx.class, "ARM1");
        ARM2 = hardwareMap.get(DcMotorEx.class, "ARM2");
        ARM1.setDirection(DcMotorEx.Direction.REVERSE);
        ARM2.setDirection(DcMotorEx.Direction.REVERSE);
    }
    private void initArmProfile(int armNumber, double targetPos) {
        if (armNumber == 1) {
            arm1StartPos = ARM1.getCurrentPosition();
            arm1TargetPos = targetPos;
            double dTotal = Math.abs(arm1TargetPos - arm1StartPos);
            arm1DDec = (V_MAX * V_MAX) / (2 * A_DEC);
            arm1Decelerating = (dTotal <= arm1DDec);
        } else {
            arm2StartPos = ARM2.getCurrentPosition();
            arm2TargetPos = targetPos;
            double dTotal = Math.abs(arm2TargetPos - arm2StartPos);
            arm2DDec = (V_MAX * V_MAX) / (2 * A_DEC);
            arm2Decelerating = (dTotal <= arm2DDec);
        }
    }
    private void ARM_SetTargets() {
        if (Target1 != userTarget1) {
            Target1 = userTarget1;
            initArmProfile(1, Target1*ticks_in_degree_1);
        }
        if (Target2 != userTarget2){
            Target2 = userTarget2;
            initArmProfile(2, Target2*ticks_in_degree_2);
        }

        // Update ARM1 target
        if (Math.abs(ARM1.getCurrentPosition() - arm1TargetPos) > 5) {
            updateArmProfile(1);
        }

        // Update ARM2 target
        if (Math.abs(ARM2.getCurrentPosition() - arm2TargetPos) > 5) {
            updateArmProfile(2);
        }
    }
    private void updateArmProfile(int armNumber) {
        DcMotorEx motor;
        double currentPos, targetPos, dDec;
        boolean decelerating;
        if (armNumber == 1){
            motor = ARM1;
            currentPos = motor.getCurrentPosition();
            targetPos = arm1TargetPos;
            decelerating = arm1Decelerating;
            dDec = arm1DDec;
        }
        else {
            motor = ARM2;
            currentPos = motor.getCurrentPosition();
            targetPos = arm2TargetPos;
            decelerating = arm2Decelerating;
            dDec = arm2DDec;
        }
        double dRemaining = Math.abs(targetPos - currentPos);
        double direction = Math.signum(targetPos - currentPos);

        // Check if we need to start decelerating
        if (!decelerating && dRemaining <= dDec) {
            decelerating = true;
            if (armNumber == 1) arm1Decelerating = true;
            else arm2Decelerating = true;
        }

        // Calculate desired velocity
        double vDesired;
        if (decelerating) {
            vDesired = Math.sqrt(2 * A_DEC * dRemaining);
            vDesired = Math.min(vDesired, V_MAX);
        } else {
            vDesired = V_MAX;
        }

        // Update target position
        double delta = vDesired * direction * LOOP_TIME;
        double newTarget = currentPos + delta;

        if (armNumber == 1) {
            target1 = newTarget / ticks_in_degree_1; // Convert to degrees
        } else {
            target2 = newTarget / ticks_in_degree_2;
        }
    }
    @Override
    public void loop(){
        ARM_SetTargets();
        double theta1_actual = Math.toRadians(target1 + ARM1_OFFSET);
        double theta2_actual = Math.toRadians(target1 + ARM1_OFFSET + target2 + ARM2_OFFSET);
        controller1.setPID(p1,i1,d1);
        int arm1Pos = ARM1.getCurrentPosition();
        double pid1 = controller1.calculate(arm1Pos,(int)(target1*ticks_in_degree_1)); //PID calculation
        double ff1 = (m1 * x1 * Math.cos(theta1_actual) + m2 * (L1 * Math.cos(theta1_actual) + x2 * Math.cos(theta2_actual))) * f1;
        double power1 = pid1 + ff1;
        ARM1.setPower(power1); //set the power

        controller2.setPID(p2,i2,d2);
        int arm2Pos = ARM2.getCurrentPosition();
        double pid2 = controller2.calculate(arm2Pos, (int)(target2*ticks_in_degree_2));
        double ff2 = m2 * x2 * Math.cos(theta2_actual) * f2;
        double power2 = pid2 + ff2;
        ARM2.setPower(power2);

        telemetry.addData("ARM1POS",arm1Pos);
        telemetry.addData("ARM1Target",(int)(target1*ticks_in_degree_1));
        telemetry.addData("ARM1Power", ARM1.getPower());
        telemetry.addData("arm1decelerating", arm1Decelerating);
        telemetry.addData("ARM2POS",arm2Pos);
        telemetry.addData("ARM2Target",(int)(target2*ticks_in_degree_2));
        telemetry.addData("ARM2Power", ARM2.getPower());
        telemetry.addData("arm2decelerating", arm2Decelerating);
        telemetry.update();
    }
}
