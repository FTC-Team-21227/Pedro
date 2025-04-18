package org.firstinspires.ftc.teamcode.subsystems;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.ArmFeedforward;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

public class ARM2 extends Subsystem {
    // BOILERPLATE
    public static final ARM2 INSTANCE = new ARM2();
    private ARM2() { }

    // USER CODE
    public MotorEx motor;
    //PIDF gains
    double p2 = TunePID.p2, i2 = TunePID.i2, d2 = TunePID.d2;
    double f2 = TunePID.f2;
    //ticks to degrees conversion
    private final double ticks_in_degree_2 = 11.2855555556;
    //length, COM, mass values for feedforward calculation (not even performed in arm2)
    private final double L1 = 43.2;
    private final double L2 = 43.2;
    private final double x1 = 36.96;
    private final double x2 = 26.4;
    private final double m1 = 810;
    private final double m2 = 99.79;
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

    public PIDFController controller = new PIDFController(p2, i2, d2, new ArmFeedforward(f2, ticks_in_degree_1 -> 1.0));

    public String name = "ARM2";

    @Override
    public Command getDefaultCommand() {
        return new HoldPosition(motor, controller, this);
    }
    public Command toHighBasket() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                highBasket2, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toHighRung() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                highRung2, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toHighRung2() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                highRung2_2, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toHighRung2First() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                highRung2_2, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toWall() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                wall2, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toWall2() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                wall2_2, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toLowBasket() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                lowBasket2, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toFloor() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                floor2, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toDown() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                down2, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toSub() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                sub2, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toVertSub() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                vertSub2, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toVertFloor() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                vertFloor2, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    @Override
    public void initialize() {
        motor = new MotorEx(name);
    }

}
