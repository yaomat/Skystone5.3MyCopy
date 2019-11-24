package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name="VuforiaSkystoneRed", group ="Concept")

public class VuforiaSkystoneRed extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFront_drive = null;
    private DcMotor leftFront_drive = null;
    private DcMotor leftBack_drive = null;
    private DcMotor rightBack_drive = null;
    //Servo2 is the grabber
    private Servo Servo2 = null;
    //Servo1 is the arm
    private Servo Servo1 = null;
    //Servo0 is the extender
    private Servo Servo0 = null;
    //next two are foundation hooks
    private Servo Servo3 = null;
    private Servo Servo4 = null;
    double runtime1 = 0;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    WebcamName Webcam1 = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    private static final String VUFORIA_KEY =
            "AQsl+Kb/////AAABmbE31v+dqUMNrHZvXmTH1TYMKPOzGdwPeKVPFFdmQC1IslYsLtjPjtKBkJ0UYlfipscEJ+KhpMxZshIN22jTxZIKJp/CIxjik5UGibWOLsfPTkMIX2WFXb7uJBFeEUr3kqLZWmrf5sAkMa9B5HNOXGKeYyqOFRBht5k0MrFIrYZAnmha98dsxodvP8TPkHY/EHy57K2ww9TCqEstOJVf6DtJO+zgMEJz8iv2ASr3Mc6RFwNvS/l1Gsq3RmpEzK3/BasvQ0gakJO4zUlMZ5+CDHchljWOcUTdls6dSeyfMxv7kkLStV018qcq15bTEK17lEbmPU7fcf97/Sp7YBe89Kw/CrVRmHQWMgSmrc+6NG6T";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;
    public boolean complete = false;
    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    @Override public void runOpMode() {
        //configuration of robot stuff
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        Webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        rightFront_drive = hardwareMap.get(DcMotor.class, "rightFront_drive");
        leftFront_drive = hardwareMap.get(DcMotor.class, "leftFront_drive");
        leftBack_drive = hardwareMap.get(DcMotor.class, "leftBack_drive");
        rightBack_drive = hardwareMap.get(DcMotor.class, "rightBack_drive");
        Servo2 = hardwareMap.get(Servo.class, "Servo2");
        Servo1 = hardwareMap.get(Servo.class, "Servo1");
        Servo0 = hardwareMap.get(Servo.class, "Servo0");
        Servo3 = hardwareMap.get(Servo.class, "Servo3");
        Servo4 = hardwareMap.get(Servo.class, "Servo4");
        rightFront_drive.setDirection(DcMotor.Direction.FORWARD);
        leftFront_drive.setDirection(DcMotor.Direction.REVERSE);
        leftBack_drive.setDirection(DcMotor.Direction.FORWARD);
        rightBack_drive.setDirection(DcMotor.Direction.REVERSE);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        //Vuforia configuration
        parameters.vuforiaLicenseKey = "AQsl+Kb/////AAABmbE31v+dqUMNrHZvXmTH1TYMKPOzGdwPeKVPFFdmQC1IslYsLtjPjtKBkJ0UYlfipscEJ+KhpMxZshIN22jTxZIKJp/CIxjik5UGibWOLsfPTkMIX2WFXb7uJBFeEUr3kqLZWmrf5sAkMa9B5HNOXGKeYyqOFRBht5k0MrFIrYZAnmha98dsxodvP8TPkHY/EHy57K2ww9TCqEstOJVf6DtJO+zgMEJz8iv2ASr3Mc6RFwNvS/l1Gsq3RmpEzK3/BasvQ0gakJO4zUlMZ5+CDHchljWOcUTdls6dSeyfMxv7kkLStV018qcq15bTEK17lEbmPU7fcf97/Sp7YBe89Kw/CrVRmHQWMgSmrc+6NG6T";
        parameters.cameraName = Webcam1;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);


        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 8.5f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 4.25f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = -2f * mmPerInch;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        //Let all the trackable listeners know where the phone is
        for (VuforiaTrackable trackable: allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        //vuforia configuration stops here
        //reset servos
        Servo2.setPosition(1);
        waitForStart();
        runtime.reset();
        forward(0.25);
        waitfor(1.075);
        strafeleft(0.17);

        targetsSkyStone.activate();
        while (!isStopRequested()) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            runtime1 = runtime.time();
            telemetry.addData("Time run:", runtime1);
            //if it doesn't find a stone within 12 seconds
            if (runtime.time() > 12){
                if (complete == false){
                    straferight(0.75);
                    waitfor(1.45);
                    brakemotors();
                    complete = true;

                }

            }
            for (VuforiaTrackable trackable : targetsSkyStone) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                brakemotors();

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                telemetry.addData("Skystone", "Visible");
                complete = true;
                //back up
                forward(-0.25);
                waitfor(0.25);
                brakemotors();
                //extend
                Servo2.setPosition(1);
                Servo1.setPosition(0.975);
                waitfor(0.5);
                //forward and align
                forward(0.25);
                waitfor(0.375);
                brakemotors();
                //grab
                Servo2.setPosition(0);
                waitfor(1);
                //retract
                Servo1.setPosition(0.05);
                waitfor(0.75);
                targetsSkyStone.deactivate();
                //backwards
                forward(-0.5);
                waitfor(0.75);
                strafeleft(-0.5);
                waitfor(6-runtime1);
                brakemotors();
                Servo1.setPosition(0.975);
                waitfor(0.75);
                Servo2.setPosition(1);
                waitfor(0.5);
                Servo1.setPosition(0.05);
                waitfor(0.75);
                Servo2.setPosition(0);
                strafeleft(0.25);
                waitfor(1.25);
                brakemotors();

            }
            else {
                telemetry.addData("Visible Target", "none");
                telemetry.addData("Skystone", "None");
            }
            telemetry.update();
        }




    }

    private void forward(double power){
        rightFront_drive.setPower(power);
        leftFront_drive.setPower(power);
        leftBack_drive.setPower(power);
        rightBack_drive.setPower(power);

    }
    private void straferight(double power){
        rightFront_drive.setPower(-power);
        leftFront_drive.setPower(power);
        leftBack_drive.setPower(-power);
        rightBack_drive.setPower(power);

    }
    private void strafeleft(double power){
        rightFront_drive.setPower(power);
        leftFront_drive.setPower(-power);
        leftBack_drive.setPower(power);
        rightBack_drive.setPower(-power);

    }
    private void brakemotors(){
        leftFront_drive.setPower(0);
        rightFront_drive.setPower(0);
        leftBack_drive.setPower(0);
        rightBack_drive.setPower(0);

    }
    private void waitfor(double waittime){
        runtime.reset();
        while (runtime.time() < waittime){

        }

    }

}


