package pedroPathing.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.NullCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import pedroPathing.subsystems.*;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Auto_Left_5samp", group = "opmodes")
public class Auto_Left extends PedroOpMode {
    public Auto_Left() {
        super(ARM1.INSTANCE, ARM2.INSTANCE, CLAW.INSTANCE, INTAKE_ANGLE.INSTANCE, CLAW_ANGLE.INSTANCE, SWEEPER.INSTANCE);
    }
    private Follower follower;


    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 106.7, Math.toRadians(0));
    private final Pose forwardPose = new Pose(19,106.7,Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(17, 127.2, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(17.4, 121.7, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(17.6, 133.2, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(19,132.7,Math.toRadians(10));
    private final Pose turnPickup3Pose = new Pose(19,132.7,Math.toRadians(28));

    /** Fourth Sample from the Sub*/
    private final Pose enterSubPose = new Pose(73.500, 119.700,Math.toRadians(-90));
    private final Pose pickup4Pose = new Pose(73.500, 108.700,Math.toRadians(-90));

    /** Score Fourth Sample*/
    private final Pose score4ControlPose1 = new Pose(56.535, 122.902,Math.toRadians(-45));
    private final Pose score4ControlPose2 = new Pose(36.871, 112.660,Math.toRadians(-45));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(61.500, 101.700, Math.toRadians(-90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(61.041, 125.974, Math.toRadians(-90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path park;
    private PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, grabPickup4, turnPickup3, scorePickup1, scorePickup2, scorePickup3, scorePickup4;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(forwardPose)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .addPath(new BezierLine(new Point(forwardPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(forwardPose.getHeading(),scorePose.getHeading())
                .build();

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our turnPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        turnPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(turnPickup3Pose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), turnPickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(turnPickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(turnPickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup4 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(enterSubPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), enterSubPose.getHeading())
                .addPath(new BezierLine(new Point(enterSubPose),new Point(pickup4Pose)))
                .setConstantHeadingInterpolation(enterSubPose.getHeading())
                .build();

        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickup4Pose),new Point(score4ControlPose1),new Point(score4ControlPose2),new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup4Pose.getHeading(),scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    public Command routine(){
        return new SequentialGroup(
            new ParallelGroup(
                INTAKE_ANGLE.INSTANCE.RotatePosition0_basket(),
                CLAW.INSTANCE.closeClaw(),
                CLAW_ANGLE.INSTANCE.backward().afterTime(0.5),
                ARM1.INSTANCE.toHighBasket(),
                ARM2.INSTANCE.toHighBasket(),
                new FollowPath(scorePreload)
            ),
            CLAW.INSTANCE.openClaw(),
            new ParallelGroup(
                INTAKE_ANGLE.INSTANCE.RotatePosition0_left().afterTime(0.5),
                CLAW_ANGLE.INSTANCE.forward().afterTime(0.5),
                ARM1.INSTANCE.toFloor().afterTime(0.3),
                ARM2.INSTANCE.toFloor().afterTime(0.3),
                new FollowPath(grabPickup1).afterTime(0.5)
            ),
            CLAW.INSTANCE.closeClaw(),
            new ParallelGroup(
                INTAKE_ANGLE.INSTANCE.RotatePosition0_basket().afterTime(1.5),
                CLAW_ANGLE.INSTANCE.backward().afterTime(1),
                ARM1.INSTANCE.toHighBasket().afterTime(0.3),
                ARM2.INSTANCE.toHighBasket().afterTime(0.3),
                new FollowPath(scorePickup1).afterTime(1)
            ),
            CLAW.INSTANCE.openClaw(),
            new ParallelGroup(
                INTAKE_ANGLE.INSTANCE.RotatePosition0_left().afterTime(0.5),
                CLAW_ANGLE.INSTANCE.forward().afterTime(0.5),
                ARM1.INSTANCE.toFloor().afterTime(0.3),
                ARM2.INSTANCE.toFloor().afterTime(0.3),
                new FollowPath(grabPickup2).afterTime(0.5)
            ),
            CLAW.INSTANCE.closeClaw(),
            new ParallelGroup(
                INTAKE_ANGLE.INSTANCE.RotatePosition0_basket().afterTime(1.5),
                CLAW_ANGLE.INSTANCE.backward().afterTime(1),
                ARM1.INSTANCE.toHighBasket().afterTime(0.3),
                ARM2.INSTANCE.toHighBasket().afterTime(0.3),
                new FollowPath(scorePickup2).afterTime(1)
            ),
            CLAW.INSTANCE.openClaw(),
            new ParallelGroup(
                INTAKE_ANGLE.INSTANCE.RotatePosition0_left().afterTime(0.5),
                CLAW_ANGLE.INSTANCE.forward().afterTime(0.5),
                ARM1.INSTANCE.toWall().afterTime(0.3),
                ARM2.INSTANCE.toWall().afterTime(0.3),
                ARM1.INSTANCE.toFloor(0.25).afterTime(2.8),
                ARM2.INSTANCE.toFloor().afterTime(2.8),
                new FollowPath(grabPickup3).afterTime(0.5),
                new FollowPath(turnPickup3).afterTime(2)
            ),
            CLAW.INSTANCE.closeClaw(),
            new ParallelGroup(
                INTAKE_ANGLE.INSTANCE.RotatePosition0_basket().afterTime(1.5),
                CLAW_ANGLE.INSTANCE.backward().afterTime(1),
                ARM1.INSTANCE.toHighBasket().afterTime(0.3),
                ARM2.INSTANCE.toHighBasket().afterTime(0.3),
                new FollowPath(scorePickup3).afterTime(1)
            ),
            CLAW.INSTANCE.openClaw(),
            new ParallelGroup(
                new FollowPath(grabPickup4).afterTime(0.5),
                CLAW_ANGLE.INSTANCE.forward().afterTime(0.5),
                CLAW.INSTANCE.openClawMore().afterTime(0.5),
                INTAKE_ANGLE.INSTANCE.RotatePosition2().afterTime(2),
                ARM1.INSTANCE.toVertSub().afterTime(0.3),
                ARM2.INSTANCE.toVertSub().afterTime(0.3),
                ARM1.INSTANCE.toVertFloor(0.25).afterTime(4)
            ),
            CLAW.INSTANCE.closeClaw(),
            new ParallelGroup(
                new FollowPath(scorePickup4).afterTime(1),
                INTAKE_ANGLE.INSTANCE.RotatePosition0_basket().afterTime(0.3),
                CLAW_ANGLE.INSTANCE.backward().afterTime(1.5),
                ARM1.INSTANCE.toVertSub().afterTime(0.3),
                ARM1.INSTANCE.toHighBasket().afterTime(2),
                ARM2.INSTANCE.toHighBasket().afterTime(2)
            ),
            CLAW.INSTANCE.openClaw(),
            new ParallelGroup(
                new FollowPath(park).afterTime(0.5),
                INTAKE_ANGLE.INSTANCE.RotatePosition0_left().afterTime(0.8),
                CLAW_ANGLE.INSTANCE.forward().afterTime(0.8),
                ARM1.INSTANCE.toWallWall().afterTime(0.3)
            )
        );
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void onInit() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void onStartButtonPressed() {
        routine().invoke();
    }

    @Override
    public void onUpdate(){
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}

