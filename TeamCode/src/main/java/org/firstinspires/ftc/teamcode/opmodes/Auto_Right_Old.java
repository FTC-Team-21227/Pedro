package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.ARM1;
import org.firstinspires.ftc.teamcode.subsystems.ARM2;
import org.firstinspires.ftc.teamcode.subsystems.CLAW;
import org.firstinspires.ftc.teamcode.subsystems.CLAW_ANGLE;
import org.firstinspires.ftc.teamcode.subsystems.INTAKE_ANGLE;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.SWEEPER;

@Autonomous(name = "Auto_Right_5+1_Old", group = "opmodes")
public class Auto_Right_Old extends PedroOpMode {
        public Auto_Right_Old() {
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
        private final Pose startPose = new Pose(9.000, 64.800, Math.toRadians(180));
        private final Pose scorePreloadPose = new Pose(34.300, 76.800,Math.toRadians(180));
        private final Pose scorePreloadControlPose = new Pose(20.893, 77.018,Math.toRadians(180));
        private final Pose leaveSubPose = new Pose(56.400, 39.000,Math.toRadians(180));
        private final Pose leaveSubControlPose = new Pose(14.339, 39.000, Math.toRadians(180));

        /** Lowest (First) Sample from the Spike Mark */
        private final Pose push1Pose = new Pose(56.400, 26.000, Math.toRadians(180));
        private final Pose push1ControlPose1 = new Pose(67.000, 39.000, Math.toRadians(180));

        private final Pose push1ControlPose2 = new Pose(67.000, 26.000, Math.toRadians(180));
        private final Pose pushed1Pose = new Pose(21.000, 26.000, Math.toRadians(180));

        /** Middle (Second) Sample from the Spike Mark */
        private final Pose push2Pose = new Pose(56.400, 16.000, Math.toRadians(180));
        private final Pose push2ControlPose1 = push1ControlPose2;
        private final Pose push2ControlPose2 = new Pose(67.000, 16.000, Math.toRadians(180));
        private final Pose pushed2Pose = new Pose(21.000, 16.000, Math.toRadians(180));

        /** Highest (Third) Sample from the Spike Mark */
        private final Pose push3Pose = new Pose(56.400, 6.000,Math.toRadians(180));
        private final Pose push3ControlPose1 = push2ControlPose2;
        private final Pose push3ControlPose2 = new Pose(67.000, 6.000,Math.toRadians(180));
        private final Pose pushed3Pose = new Pose(14.550, 6.000, Math.toRadians(180));

        /** Control Pose for Scoring Second Specimen*/
        private final Pose score2Pose = new Pose(34.300, 74.800,Math.toRadians(180));
        private final Pose score2ControlPose = new Pose(14.550, 74.800,Math.toRadians(180));

        /** Cycling Specimens*/
        private final Pose pickupPose = new Pose(14.550, 34.700,Math.toRadians(180));
        private final Pose pickupControlPose = new Pose(19.000, 74.800, Math.toRadians(180));
        private final Pose scorePose = new Pose(34.300, 72.800, Math.toRadians(180));
        private final Pose scoreControlPose = new Pose(19.000, 72.800,Math.toRadians(180));

        /** +1 sample pose for our robot */
        private final Pose scoreSamplePose = new Pose(18.230, 124.950, Math.toRadians(315));

        /* These are our Paths and PathChains that we will define in buildPaths() */
        private Path scorePreload, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, grabPickup4, scorePickup4, grabSample, scoreSample;
        private PathChain pushSamples;

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

                /* This is our scorePreload path. We are using a BezierCurve. */
                scorePreload = new Path(new BezierCurve(new Point(startPose), new Point(scorePreloadControlPose), new Point(scorePreloadPose)));
                scorePreload.setConstantHeadingInterpolation(startPose.getHeading());

                /* Pushing samples PathChain */
                pushSamples = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(scorePreloadPose), new Point(leaveSubControlPose), new Point(leaveSubPose)))
                        .addPath(new BezierCurve(new Point(leaveSubPose), new Point(push1ControlPose1), new Point(push1ControlPose2), new Point(push1Pose)))
                        .addPath(new BezierLine(new Point(push1Pose), new Point(pushed1Pose)))
                        .addPath(new BezierCurve(new Point(push1Pose), new Point(push2ControlPose1), new Point(push2ControlPose2), new Point(push2Pose)))
                        .addPath(new BezierLine(new Point(push2Pose), new Point(pushed2Pose)))
                        .addPath(new BezierCurve(new Point(push2Pose), new Point(push3ControlPose1), new Point(push3ControlPose2), new Point(push3Pose)))
                        .addPath(new BezierLine(new Point(push3Pose), new Point(pushed3Pose)))
                        .setConstantHeadingInterpolation(scorePreloadPose.getHeading())
                        .build();


                /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
                scorePickup1 = new Path(new BezierCurve(new Point(pushed3Pose), new Point(score2ControlPose), new Point(score2Pose)));
                scorePickup1.setConstantHeadingInterpolation(push3Pose.getHeading());

                /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
                grabPickup2 = new Path(new BezierCurve(new Point(score2Pose), new Point(pickupControlPose), new Point(pickupPose)));
                grabPickup2.setConstantHeadingInterpolation(score2Pose.getHeading());

                /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
                scorePickup2 = new Path(new BezierCurve(new Point(pickupPose), new Point(scoreControlPose), new Point(scorePose)));
                scorePickup2.setConstantHeadingInterpolation(pickupPose.getHeading());

                /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
                grabPickup3 = new Path(new BezierCurve(new Point(scorePose), new Point(pickupControlPose), new Point(pickupPose)));
                grabPickup3.setConstantHeadingInterpolation(scorePose.getHeading());

                /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
                scorePickup3 = new Path(new BezierCurve(new Point(pickupPose), new Point(scoreControlPose), new Point(scorePose)));
                scorePickup3.setConstantHeadingInterpolation(pickupPose.getHeading());

                /* This is our grabPickup4 PathChain. We are using a single path with a BezierLine, which is a straight line. */
                grabPickup4 = new Path(new BezierCurve(new Point(scorePose), new Point(pickupControlPose), new Point(pickupPose)));
                grabPickup4.setConstantHeadingInterpolation(scorePose.getHeading());

                scorePickup4 = new Path(new BezierCurve(new Point(pickupPose), new Point(scoreControlPose), new Point(scorePose)));
                scorePickup4.setConstantHeadingInterpolation(pickupPose.getHeading());

                grabSample = new Path(new BezierCurve(new Point(scorePose), new Point(pickupControlPose), new Point(pickupPose)));
                grabSample.setConstantHeadingInterpolation(scorePose.getHeading());

                scoreSample = new Path(new BezierLine(new Point(pickupPose), new Point(scoreSamplePose)));
                scoreSample.setLinearHeadingInterpolation(pickupPose.getHeading(), scoreSamplePose.getHeading());
        }

        public Command routine(){
                //TODO: change wall2() and angles to floor and flat
                return new SequentialGroup(
                        new ParallelGroup(
                                CLAW.INSTANCE.closeClaw(),
                                INTAKE_ANGLE.INSTANCE.RotatePosition0(),
                                ARM1.INSTANCE.toHighRung2().afterTime(0.15),
                                ARM2.INSTANCE.toHighRung2First().afterTime(0.15),
                                new FollowPath(scorePreload)
                        ),
                        new ParallelGroup(
                                CLAW.INSTANCE.openClawMore(),
                                new FollowPath(pushSamples),
                                ARM1.INSTANCE.toDown().afterTime(1),
                                ARM2.INSTANCE.toDown().afterTime(1),
                                ARM1.INSTANCE.toWall2().afterTime(5),
                                ARM2.INSTANCE.toWall2().afterTime(5),
                                CLAW_ANGLE.INSTANCE.backward().afterTime(4.8),
                                INTAKE_ANGLE.INSTANCE.RotatePositionNegative1().afterTime(5.5),
                                SWEEPER.INSTANCE.RotatePosition0().afterTime(6.5)
                        ),
                        CLAW.INSTANCE.closeClaw(),
                        new ParallelGroup(
                                SWEEPER.INSTANCE.RotatePosition1().afterTime(0.5),
                                ARM1.INSTANCE.toHighRung2().afterTime(0.3),
                                ARM2.INSTANCE.toHighRung2().afterTime(0.3),
                                new FollowPath(scorePickup1).afterTime(0.4),
                                CLAW_ANGLE.INSTANCE.forward().afterTime(0.5),
                                INTAKE_ANGLE.INSTANCE.RotatePosition0().afterTime(0.4)
                        ),
                        new ParallelGroup(
                                CLAW.INSTANCE.openClawMore(),
                                new FollowPath(grabPickup2),
                                ARM1.INSTANCE.toWall2().afterTime(0.4), //0.5
                                ARM2.INSTANCE.toWall2().afterTime(0.4),
                                INTAKE_ANGLE.INSTANCE.RotatePositionNegative1().afterTime(0.7),
                                CLAW_ANGLE.INSTANCE.backward()
                        ),
                        CLAW.INSTANCE.closeClaw(),
                        new ParallelGroup(
                                SWEEPER.INSTANCE.RotatePosition1().afterTime(0.5),
                                ARM1.INSTANCE.toHighRung2().afterTime(0.3),
                                ARM2.INSTANCE.toHighRung2().afterTime(0.3),
                                new FollowPath(scorePickup2).afterTime(0.4),
                                INTAKE_ANGLE.INSTANCE.RotatePosition0().afterTime(0.4),
                                CLAW_ANGLE.INSTANCE.forward().afterTime(0.5)
                                ),
                        new ParallelGroup(
                                CLAW.INSTANCE.openClawMore(),
                                new FollowPath(grabPickup3),
                                ARM1.INSTANCE.toWall2().afterTime(0.4), //0.5
                                ARM2.INSTANCE.toWall2().afterTime(0.4),
                                INTAKE_ANGLE.INSTANCE.RotatePositionNegative1().afterTime(0.7),
                                CLAW_ANGLE.INSTANCE.backward()
                        ),
                        CLAW.INSTANCE.closeClaw(),
                        new ParallelGroup(
                                SWEEPER.INSTANCE.RotatePosition1().afterTime(0.5),
                                ARM1.INSTANCE.toHighRung2().afterTime(0.3),
                                ARM2.INSTANCE.toHighRung2().afterTime(0.3),
                                new FollowPath(scorePickup3).afterTime(0.4),
                                INTAKE_ANGLE.INSTANCE.RotatePosition0().afterTime(0.4),
                                CLAW_ANGLE.INSTANCE.forward().afterTime(0.5)
                        ),
                        new ParallelGroup(
                                CLAW.INSTANCE.openClawMore(),
                                new FollowPath(grabPickup4),
                                ARM1.INSTANCE.toWall2().afterTime(0.4), //0.5
                                ARM2.INSTANCE.toWall2().afterTime(0.4),
                                INTAKE_ANGLE.INSTANCE.RotatePositionNegative1().afterTime(0.7),
                                CLAW_ANGLE.INSTANCE.backward()
                        ),
                        CLAW.INSTANCE.closeClaw(),
                        new ParallelGroup(
                                SWEEPER.INSTANCE.RotatePosition1().afterTime(0.5),
                                ARM1.INSTANCE.toHighRung2().afterTime(0.3),
                                ARM2.INSTANCE.toHighRung2().afterTime(0.3),
                                new FollowPath(scorePickup4).afterTime(0.4),
                                INTAKE_ANGLE.INSTANCE.RotatePosition0().afterTime(0.4),
                                CLAW_ANGLE.INSTANCE.forward().afterTime(0.5)
                        ),
                        new ParallelGroup(
                                CLAW.INSTANCE.openClawMore(),
                                new FollowPath(grabSample),
                                ARM1.INSTANCE.toWall2().afterTime(0.4), //0.5
                                ARM2.INSTANCE.toWall2().afterTime(0.4),
                                INTAKE_ANGLE.INSTANCE.RotatePositionNegative1().afterTime(0.7),
                                CLAW_ANGLE.INSTANCE.backward()
                        ),
                        CLAW.INSTANCE.closeClaw(),
                        new ParallelGroup(
                                ARM1.INSTANCE.toHighBasket().afterTime(0.3),
                                ARM2.INSTANCE.toHighBasket().afterTime(0.3),
                                new FollowPath(scoreSample),
//                                CLAW_ANGLE.INSTANCE.backward().afterTime(0.3),
                                INTAKE_ANGLE.INSTANCE.RotatePosition0_basket().afterTime(1)
                        ),
                        CLAW.INSTANCE.openClaw()
                );
        }

        /** This method is called once at the init of the OpMode. **/
        @Override
        public void onInit() {
                follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
                follower.setStartingPose(startPose);
                buildPaths();
                new ParallelGroup(
                        CLAW.INSTANCE.closeClaw(),
                        INTAKE_ANGLE.INSTANCE.RotatePosition1(),
                        CLAW_ANGLE.INSTANCE.forward()
                ).invoke();
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

        @Override
        public void onStop(){
                PoseStorage.currentPose = follower.getPose();
        }
}