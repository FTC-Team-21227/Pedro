package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

public class ARM1_V2 extends Subsystem {
    // BOILERPLATE
    public static final ARM1_V2 INSTANCE = new ARM1_V2();
    private ARM1_V2() { }

    // USER CODE
    public MotorEx motor;

    //PIDF gains
    double p1 = TunePID_MotionProfile.p1, i1 = TunePID_MotionProfile.i1, d1 = TunePID_MotionProfile.d1;
    double f1 = TunePID_MotionProfile.f1;
    //ticks to degrees conversion
    private final double ticks_in_degree_1 = TunePID_MotionProfile.ticks_in_degree_1;
    private final double highBasket = Subsystem_Constants.highBasket1*ticks_in_degree_1;
    private final double highRung = Subsystem_Constants.highRung1*ticks_in_degree_1;
    private final double highRung2 = Subsystem_Constants.highRung1_2*ticks_in_degree_1;
    private final double wall = Subsystem_Constants.wall1*ticks_in_degree_1;
    private final double wallwall = Subsystem_Constants.wallwall1*ticks_in_degree_1;
    private final double wall2 = Subsystem_Constants.wall1_2*ticks_in_degree_1;
    private final double lowBasket = Subsystem_Constants.lowBasket1*ticks_in_degree_1;
    private final double floor = Subsystem_Constants.floor1*ticks_in_degree_1;
    private final double down = Subsystem_Constants.down1*ticks_in_degree_1;
    private final double sub = Subsystem_Constants.sub1*ticks_in_degree_1;
    private final double vertSub1 = Subsystem_Constants.vertSub1*ticks_in_degree_1;
    private final double vertFloor1 = Subsystem_Constants.vertFloor1*ticks_in_degree_1;
    public PIDFController controller2 = new PIDFController(p1, i1, d1, new Arm1Feedforward(f1));

    public String name = "ARM1";

    // Profile Parameters (tune these!)
    private final double V_MAX = TunePID_MotionProfile.V_MAX; // Encoder ticks/sec (≈ 100° /sec if 1 tick/degree)
    private final double A_DEC = TunePID_MotionProfile.A_DEC; // Ticks/sec² (adjust for smooth stopping)
    private ProfiledPIDController controller = new ProfiledPIDController(p1, i1, d1, new TrapezoidProfile.Constraints(V_MAX,A_DEC));
    private Arm1Feedforward feedforward = new Arm1Feedforward(f1);

    public double target1 = 0;
    final double tolerance = 5;

    @Override
    public Command getDefaultCommand() {
        return new HoldPosition(motor, controller2, this);
    }
    public Command toHighBasket() {
        return new LambdaCommand()
                .setStart(() -> {target1 = highBasket; motor.getMotor().setTargetPosition((int) target1);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target1)+feedforward.compute(target1)))
                .setIsDone(() -> Math.abs(target1 - motor.getCurrentPosition()) <= tolerance);
    }

    public Command toHighRung() {
        return new LambdaCommand()
                .setStart(() -> {target1 = highRung; motor.getMotor().setTargetPosition((int) target1);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target1)+feedforward.compute(target1)))
                .setIsDone(() -> Math.abs(target1 - motor.getCurrentPosition()) <= tolerance);
    }

    public Command toHighRung2() {
        return new LambdaCommand()
                .setStart(() -> {target1 = highRung2; motor.getMotor().setTargetPosition((int) target1);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target1)+feedforward.compute(target1)))
                .setIsDone(() -> Math.abs(target1 - motor.getCurrentPosition()) <= tolerance);
    }
    public Command toWall() {
        return new LambdaCommand()
                .setStart(() -> {target1 = wall; motor.getMotor().setTargetPosition((int) target1);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target1)+feedforward.compute(target1)))
                .setIsDone(() -> Math.abs(target1 - motor.getCurrentPosition()) <= tolerance);
    }

    public Command toWallWall() {
        return new LambdaCommand()
                .setStart(() -> {target1 = wallwall; motor.getMotor().setTargetPosition((int) target1);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target1)+feedforward.compute(target1)))
                .setIsDone(() -> Math.abs(target1 - motor.getCurrentPosition()) <= tolerance);
    }

    public Command toWall2() {
        return new LambdaCommand()
                .setStart(() -> {target1 = wall2; motor.getMotor().setTargetPosition((int) target1);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target1)+feedforward.compute(target1)))
                .setIsDone(() -> Math.abs(target1 - motor.getCurrentPosition()) <= tolerance);
    }
    public Command toLowBasket() {
        return new LambdaCommand()
                .setStart(() -> {target1 = lowBasket; motor.getMotor().setTargetPosition((int) target1);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target1)+feedforward.compute(target1)))
                .setIsDone(() -> Math.abs(target1 - motor.getCurrentPosition()) <= tolerance);
    }

    public Command toFloor() {
        return new LambdaCommand()
                .setStart(() -> {target1 = floor; motor.getMotor().setTargetPosition((int) target1);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target1)+feedforward.compute(target1)))
                .setIsDone(() -> Math.abs(target1 - motor.getCurrentPosition()) <= tolerance);
    }

    public Command toDown() {
        return new LambdaCommand()
                .setStart(() -> {target1 = down; motor.getMotor().setTargetPosition((int) target1);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target1)+feedforward.compute(target1)))
                .setIsDone(() -> Math.abs(target1 - motor.getCurrentPosition()) <= tolerance);
    }
    public Command toSub() {
        return new LambdaCommand()
                .setStart(() -> {target1 = sub; motor.getMotor().setTargetPosition((int) target1);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target1)+feedforward.compute(target1)))
                .setIsDone(() -> Math.abs(target1 - motor.getCurrentPosition()) <= tolerance);
    }

    public Command toVertSub() {
        return new LambdaCommand()
                .setStart(() -> {target1 = vertSub1; motor.getMotor().setTargetPosition((int) target1);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target1)+feedforward.compute(target1)))
                .setIsDone(() -> Math.abs(target1 - motor.getCurrentPosition()) <= tolerance);
    }

    public Command toVertFloor() {
        return new LambdaCommand()
                .setStart(() -> {target1 = vertFloor1; motor.getMotor().setTargetPosition((int) target1);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target1)+feedforward.compute(target1)))
                .setIsDone(() -> Math.abs(target1 - motor.getCurrentPosition()) <= tolerance);
    }

    @Override
    public void initialize() {
        motor = new MotorEx(name);
        motor.reverse();
        motor.resetEncoder();
    }
}
