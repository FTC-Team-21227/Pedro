package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.ArmFeedforward;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

public class ARM2_V2 extends Subsystem {
    // BOILERPLATE
    public static final ARM2_V2 INSTANCE = new ARM2_V2();
    private ARM2_V2() { }

    // USER CODE
    public MotorEx motor;
    //PIDF gains
    double p2 = TunePID_MotionProfile.p2, i2 = TunePID_MotionProfile.i2, d2 = TunePID_MotionProfile.d2;
    double f2 = TunePID_MotionProfile.f2;
    //ticks to degrees conversion
    private final double ticks_in_degree_2 = TunePID_MotionProfile.ticks_in_degree_2;
    private final double highBasket2 = Subsystem_Constants.highBasket2_auto*ticks_in_degree_2;
    final double highRung2 = Subsystem_Constants.highRung2*ticks_in_degree_2;
    final double highRung2_2 = Subsystem_Constants.highRung2_2*ticks_in_degree_2;
    private final double wall2 = Subsystem_Constants.wall2*ticks_in_degree_2;
    private final double wall2_2 = Subsystem_Constants.wall2_2*ticks_in_degree_2;
    private final double lowBasket2 = Subsystem_Constants.lowBasket2*ticks_in_degree_2;
    private final double floor2 = Subsystem_Constants.floor2*ticks_in_degree_2;
    private final double down2 = Subsystem_Constants.down2*ticks_in_degree_2;
    private final double sub2 = Subsystem_Constants.sub2*ticks_in_degree_2;
    private final double vertSub2 = Subsystem_Constants.vertSub2*ticks_in_degree_2;
    private final double vertFloor2 = Subsystem_Constants.vertFloor2*ticks_in_degree_2;

    public PIDFController controller2 = new PIDFController(p2, i2, d2, new Arm2Feedforward(f2));

    public String name = "ARM2";
    // Profile Parameters (tune these!)
    private final double V_MAX = TunePID_MotionProfile.V_MAX; // Encoder ticks/sec (≈ 100° /sec if 1 tick/degree)
    private final double A_DEC = TunePID_MotionProfile.A_DEC; // Ticks/sec² (adjust for smooth stopping)
    private ProfiledPIDController controller = new ProfiledPIDController(p2, i2, d2, new TrapezoidProfile.Constraints(V_MAX,A_DEC));
    private Arm2Feedforward feedforward = new Arm2Feedforward(f2);

    public double target2 = 0;
    final double tolerance = 5;

    @Override
    public Command getDefaultCommand() {
        return new HoldPosition(motor, controller2, this);
    }
    public Command toHighBasket() {
        return new LambdaCommand()
                .setStart(() -> {target2 = highBasket2; motor.getMotor().setTargetPosition((int) target2);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target2)+feedforward.compute(target2)))
                .setIsDone(() -> Math.abs(target2 - motor.getCurrentPosition()) <= tolerance);
    }
    public Command toHighRung() {
        return new LambdaCommand()
                .setStart(() -> {target2 = highRung2; motor.getMotor().setTargetPosition((int) target2);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target2)+feedforward.compute(target2)))
                .setIsDone(() -> Math.abs(target2 - motor.getCurrentPosition()) <= tolerance);
    }

    public Command toHighRung2() {
        return new LambdaCommand()
                .setStart(() -> {target2 = highRung2_2; motor.getMotor().setTargetPosition((int) target2);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target2)+feedforward.compute(target2)))
                .setIsDone(() -> Math.abs(target2 - motor.getCurrentPosition()) <= tolerance);
    }
    public Command toHighRung2First() {
        return new LambdaCommand()
                .setStart(() -> {target2 = highRung2_2+0.75*ticks_in_degree_2; motor.getMotor().setTargetPosition((int) target2);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target2)+feedforward.compute(target2)))
                .setIsDone(() -> Math.abs(target2 - motor.getCurrentPosition()) <= tolerance);
    }
    public Command toWall() {
        return new LambdaCommand()
                .setStart(() -> {target2 = wall2; motor.getMotor().setTargetPosition((int) target2);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target2)+feedforward.compute(target2)))
                .setIsDone(() -> Math.abs(target2 - motor.getCurrentPosition()) <= tolerance);
    }

    public Command toWall2() {
        return new LambdaCommand()
                .setStart(() -> {target2 = wall2_2; motor.getMotor().setTargetPosition((int) target2);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target2)+feedforward.compute(target2)))
                .setIsDone(() -> Math.abs(target2 - motor.getCurrentPosition()) <= tolerance);
    }
    public Command toLowBasket() {
        return new LambdaCommand()
                .setStart(() -> {target2 = lowBasket2; motor.getMotor().setTargetPosition((int) target2);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target2)+feedforward.compute(target2)))
                .setIsDone(() -> Math.abs(target2 - motor.getCurrentPosition()) <= tolerance);
    }

    public Command toFloor() {
        return new LambdaCommand()
                .setStart(() -> {target2 = floor2; motor.getMotor().setTargetPosition((int) target2);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target2)+feedforward.compute(target2)))
                .setIsDone(() -> Math.abs(target2 - motor.getCurrentPosition()) <= tolerance);
    }

    public Command toDown() {
        return new LambdaCommand()
                .setStart(() -> {target2 = down2; motor.getMotor().setTargetPosition((int) target2);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target2)+feedforward.compute(target2)))
                .setIsDone(() -> Math.abs(target2 - motor.getCurrentPosition()) <= tolerance);
    }
    public Command toSub() {
        return new LambdaCommand()
                .setStart(() -> {target2 = sub2; motor.getMotor().setTargetPosition((int) target2);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target2)+feedforward.compute(target2)))
                .setIsDone(() -> Math.abs(target2 - motor.getCurrentPosition()) <= tolerance);
    }

    public Command toVertSub() {
        return new LambdaCommand()
                .setStart(() -> {target2 = vertSub2; motor.getMotor().setTargetPosition((int) target2);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target2)+feedforward.compute(target2)))
                .setIsDone(() -> Math.abs(target2 - motor.getCurrentPosition()) <= tolerance);
    }

    public Command toVertFloor() {
        return new LambdaCommand()
                .setStart(() -> {target2 = vertFloor2; motor.getMotor().setTargetPosition((int) target2);})
                .setUpdate(() -> motor.setPower(controller.calculate(motor.getCurrentPosition(), target2)+feedforward.compute(target2)))
                .setIsDone(() -> Math.abs(target2 - motor.getCurrentPosition()) <= tolerance);
    }

    @Override
    public void initialize() {
        motor = new MotorEx(name);
        motor.reverse();
        motor.resetEncoder();
    }

}
