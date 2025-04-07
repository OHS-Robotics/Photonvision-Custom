/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot;

import static frc.robot.Constants.Vision.*;

import java.util.List;
import java.util.stream.Collectors;

import javax.xml.transform.OutputKeys;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Robot extends TimedRobot {
    // private SwerveDrive drivetrain; // CODA commenting this out for now since this is just running on the test bed
    // private VisionSim visionSim; //CODA commenting this out too since it's just used for simulating a camera in the simulation
                                    //Since we're using a real camera, I'll comment this out to avoid confusion
    private PhotonCamera camera; //CODA this is the real camera object that you will use

    private final double VISION_TURN_kP = 0.01; //CODA This is just a constant that scales how fast the robot would turn to face a new vision target

    private XboxController controller;

    @Override
    public void robotInit() {
        // drivetrain = new SwerveDrive();
        camera = new PhotonCamera(kCameraName); //CODA the camera object was declared above, but here is where it's actually initialized.  
                                                // You may need to change that kCameraName variable to match something you've set to it?
                                                // Pro Tip: click on a variable and right click -> go to definition to see whewre it's created (or press F12)                                  
        // visionSim = new VisionSim(camera);

            // Optional: Add an initial Shuffleboard entry
            Shuffleboard.getTab("Vision").addDouble("AprilTag ID", this::getAprilTagID);
        }
    
        /**
         * Get the AprilTag ID from the best target.
         */
        public double getAprilTagID() {
            PhotonPipelineResult result = camera.getLatestResult();
            if (result.hasTargets()) {
                return result.getBestTarget().getFiducialId();
            }
            return -1; // No tag found
        }
        {
        controller = new XboxController(0);
    }

    @Override
    public void robotPeriodic() {
        // Update drivetrain subsystem
        // drivetrain.periodic();

        // Log values to the dashboard
        // drivetrain.log();
    }

    @Override
    public void disabledPeriodic() {
        // drivetrain.stop();
    }

    @Override
    public void teleopInit() {
        resetPose();
    }

    @Override
    public void teleopPeriodic() {
        //CODA this code will only run while the robot is in teleop mode and enabled.  If you want code that runs all the time, putting it in robotPeriodic should do that (I'm not positive though--that may run only when enabled as well.  You could test that out)

        // Calculate drivetrain commands from Joystick values
        // CODA here, the forward, strafe, and turn values are gathered, based on joystick input, which would normally be sent to the swerve drive (if we hadn't commented it out) 
        double forward = -controller.getLeftY() * Constants.Swerve.kMaxLinearSpeed;
        double strafe = -controller.getLeftX() * Constants.Swerve.kMaxLinearSpeed;
        double turn = -controller.getRightX() * Constants.Swerve.kMaxAngularSpeed;


        // Read in relevant data from the Camera

        // CODA here is probably the most relevant code for you.  this is where we're checking whether the camera actually sees any targets
        // See my commends for more explanation
        boolean targetVisible = false;
        double targetYaw = 0.0;
        var results = camera.getAllUnreadResults(); //CODA here we create a new variable called results and assign to it the camera's results by calling the function "getAllUnreadResults()"
        if (!results.isEmpty()) { //CODA here we check if the results are not empty (the "isEmpty()" function returns a boolean value (true/false only) and the "!" means "not")
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1); //CODA here, we get the last one in the list (lists in programming are almost always zero-based index).  So if the list is 4 long, element 0 would be the first, and 3 would be the last
            if (result.hasTargets()) { //CODA see if the result that we gathered saw at least one target
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) { //CODA here, we use a special "for" loop.  This basically calls the function "getTargets()", which returns a list.  We then run this "for" loop once for each element in that list
                    if (target.getFiducialId() == 7) { //CODA here, we're specifically checking if the target we're currently checking is equal to 7 (double equals sign means "equivalance". Basically checking if two things are equal, rather than assigning a value to a variable)
                        // CODA you may want to change the line above to be something other than 7, or you may want to remove that IF statement entirely and replace it with what I wrote (and commented out) on line 110, which just prints the ID of every target it sees
                        // Found Tag 7, record its information
                        targetYaw = target.getYaw(); //CODA here, we get the YAW.  Just like there is XYZ coordinates for translational movement, for rotations, we have "yaw" "pitch" and "roll"
                        //CODA imagine a plane.  Pitch is like the nose of the plane tilting up and down.  Roll is like doing a barrel roll.  Yaw is like the plane staying flat and turning to look to its right.  Like how our robot can spin
                        targetVisible = true; //CODA here we set the targetVisible variable to true, so that we can remember it for later
                    }
                    // if (target.getFiducialId() != -1) { //CODA if you hover over the getFiducialID function, it tells you that it returns -1 if the fiducial ID is not set for the target.  So we want to check that.
                                                            //If we didn't check that and then, say, passed along this fiducial ID to some other piece of code that did something with it, then it could crash that code if it wasn't expecting the possibility of a negative number
                    //     System.out.println("Saw a target (ID = " + target.getFiducialId() + ")");
                    // }
                }
            }
        }

        // Auto-align when requested
        if (controller.getAButton() && targetVisible) { //CODA here, if the controller's A button is pressed AND we saw target #7, then we override the turn input that we had gathered from the controller above
            // Driver wants auto-alignment to tag 7
            // And, tag 7 is in sight, so we can turn toward it.
            // Override the driver's turn command with an automatic one that turns toward the tag.
            turn = -1.0 * targetYaw * VISION_TURN_kP * Constants.Swerve.kMaxAngularSpeed;
        }

        // Command drivetrain motors based on target speeds
        // drivetrain.drive(forward, strafe, turn);

        // Put debug information to the dashboard
        SmartDashboard.putBoolean("Vision Target Visible", targetVisible); //CODA this just puts an element on the smart dashboard (or shuffleboard) that will light up green/red based on whether it sees a target.  This could be very useful for you
        
        //CODA a note for you: two very useful tools for you will be:
        // - Putting values to shuffleboard (which you do by calling SmartDashboard.putXXX())
        // - Printing values to the driver station output window by calling System.out.println()
        // These things allow you to see whether your code is reaching a certain point, print out diagnostic messages for yourself, and much more.  Very useful to know when it seems difficult to "look inside" of the code while it's running
    }

    @Override
    public void simulationPeriodic() {
        // Update drivetrain simulation
        // drivetrain.simulationPeriodic();

        // Update camera simulation
        // visionSim.simulationPeriodic(drivetrain.getSimPose());

        // var debugField = visionSim.getSimDebugField();
        // debugField.getObject("EstimatedRobot").setPose(drivetrain.getPose());
        // debugField.getObject("EstimatedRobotModules").setPoses(drivetrain.getModulePoses());

        // Calculate battery voltage sag due to current draw
        // var batteryVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(drivetrain.getCurrentDraw());

        // Using max(0.1, voltage) here isn't a *physically correct* solution,
        // but it avoids problems with battery voltage measuring 0.
        // RoboRioSim.setVInVoltage(Math.max(0.1, batteryVoltage));
    }

    public void resetPose() {
        // Example Only - startPose should be derived from some assumption
        // of where your robot was placed on the field.
        // The first pose in an autonomous path is often a good choice.
        var startPose = new Pose2d(1, 1, new Rotation2d());
        // drivetrain.resetPose(startPose, true);
        // visionSim.resetSimPose(startPose);
    }
}
