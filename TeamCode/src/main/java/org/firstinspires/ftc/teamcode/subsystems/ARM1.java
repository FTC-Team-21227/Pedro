package org.firstinspires.ftc.teamcode.subsystems;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.ArmFeedforward;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

public class ARM1 extends Subsystem {
    // BOILERPLATE
    public static final ARM1 INSTANCE = new ARM1();
    private ARM1() { }

    // USER CODE
    public MotorEx motor;
    //PIDF gains
    double p1 = TunePID.p1, i1 = TunePID.i1, d1 = TunePID.d1;
    double f1 = TunePID.f1;
    //ticks to degrees conversion
    private final double ticks_in_degree_1 = 41.8211111111;
    //length, COM, mass values for feedforward calculation
    private final double L1 = 43.2;
    private final double L2 = 43.2;
    private final double x1 = 36.96;
    private final double x2 = 26.4;
    private final double m1 = 810;
    private final double m2 = 99.79;
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

    public PIDFController controller = new PIDFController(p1, i1, d1, new Arm1Feedforward(f1));
    public PIDFController controller2 = new PIDFController(p1, i1, d1, new Arm1Feedforward(f1));

    public String name = "ARM1";

    @Override
    public Command getDefaultCommand() {
        return new HoldPosition(motor, controller, this);
    }
    public Command toHighBasket() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                highBasket, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toHighRung() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                highRung, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toHighRung2() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                highRung2, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toWall() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                wall, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toWallWall() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                wallwall, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toWall2() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                wall2, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toLowBasket() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                lowBasket, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toFloor() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                floor, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toDown() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                down, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toSub() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                sub, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toVertSub() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                vertSub1, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toVertFloor() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                vertFloor1, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toFloor(double power) {
        controller2.setKP(controller.getKP()*power);
        return new RunToPosition(motor, // MOTOR TO MOVE
                floor, // TARGET POSITION, IN TICKS
                controller2, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toDown(double power) {
        controller2.setKP(controller.getKP()*power);
        return new RunToPosition(motor, // MOTOR TO MOVE
                down, // TARGET POSITION, IN TICKS
                controller2, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toSub(double power) {
        controller2.setKP(controller.getKP()*power);
        return new RunToPosition(motor, // MOTOR TO MOVE
                sub, // TARGET POSITION, IN TICKS
                controller2, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toVertSub(double power) {
        controller2.setKP(controller.getKP()*power);
        return new RunToPosition(motor, // MOTOR TO MOVE
                vertSub1, // TARGET POSITION, IN TICKS
                controller2, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toVertFloor(double power) {
        controller2.setKP(controller.getKP()*power);
        return new RunToPosition(motor, // MOTOR TO MOVE
                vertFloor1, // TARGET POSITION, IN TICKS
                controller2, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    @Override
    public void initialize() {
        motor = new MotorEx(name);
    }
}
