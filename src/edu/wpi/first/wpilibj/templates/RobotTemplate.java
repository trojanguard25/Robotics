package edu.wpi.first.wpilibj.templates;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;


public class RobotTemplate extends SimpleRobot {

    /*
     * Declaring the objects
     */
    //Drivers Station output box declaration
    DriverStationLCD robot = DriverStationLCD.getInstance();
    /*
     * Joysticks
     */
    Timer timer = new Timer();
    Joystick chassis1 = new Joystick(1);
    Joystick chassis2 = new Joystick(2);
    Joystick climbing1 = new Joystick(3);
    Joystick climbing2 = new Joystick(4);
    /*
     * This is used for chassis (the wheels)
     *  drives the robot arround
     */
    RobotDrive chassis = new RobotDrive(1, 2);
    //climbing motors declarations
    RobotDrive climb = new RobotDrive(3, 4);
    /*
     * used for camera control
     */
    Servo servoHorizontal = new Servo(7);
    Servo servoVertical = new Servo(8);

    /*
     * used for climbing
     */
    Encoder firstEncoder = new Encoder(6, 7, false, CounterBase.EncodingType.k1X);
    Encoder secondEncoder = new Encoder(9, 10, false, CounterBase.EncodingType.k1X);
    /*
     * used for climbing
     */
    DigitalInput limitSwitch1 = new DigitalInput(12);
    DigitalInput limitSwitch2 = new DigitalInput(13);
    /*
     * The motors that will throw the frisbee
     */
    RobotDrive shooter = new RobotDrive(10, 9);
    /*
     * Controls the window motor that will be used to load the frisbees into the shooter
     */
    
    
    Victor loader = new Victor(6);
    
    
    Compressor compressor = new Compressor(2,5,2,5);
    Solenoid solenoid = new Solenoid(3,2);
    Solenoid solenoid2 = new Solenoid(3,3);
    
    /*
     * Sonar declaration
     *      *Note* sonar is declared as an AnalogChannel because the sonar
     *      can not go through the digital sidecar 
     *      (it can, it just requires more soldering)
     */
    //private AnalogChannel sonar = new AnalogChannel(5);
    /*
     * These variables are used with the Camera
     */
    private double vert = 90;
    private double hori = 90;
    private double UIVert = 90;
    private double UIHori = 90;
    /*
     * This is the declaration of the variables that will be used in the print
     * Function
     */
    double sonarDistance;
    double encoder1;
    double encoder2;
    /*
     * This is the declaration of the all the variabels that are used with the
     * spike relay
     */
    Relay relay = new Relay(5);
    public boolean driveMode = true;
    //this counter will prevent the printing screen from flashing as much
    private int counter = 0;
    private boolean shoot = false;
    
    private final int climbingHeight1=-692;
    private final int climbingHeight2=-3530;
    private final int loadingHeight1=6863;
    private final int loadingHeight2=11369;
    private final int shootingHeight1=-3904;
    private final int shootingHeight2=-4981;
    private final int blockingHeight1=-22600;
    private final int blockingHeight2=-11100;
    
    
    
    /*
     * Camera's final variabels
     */
    private final int cameraMaxAngle=179;
    private final int cameraMinAngle=1;
    int climbingCounter=0;
    
    boolean encoder1Stop=false;
    boolean encoder2Stop=false;
    
    boolean adjustLoading=false;
    boolean adjustShooting=false;
    boolean adjustClimbing=false;
    boolean adjustZero=false;
    
    final int encoderTreshold=100;
    
    int autonomousBuffer=0;
    
    boolean isShooterEnabled=false;
    boolean isLoaderEnabled=false;



    /*
     * This is automatically called at the beginning of the competition
     */
    protected void robotInit() {
        getWatchdog().kill();
        firstEncoder.setDistancePerPulse(.004);
        firstEncoder.start();
        secondEncoder.setDistancePerPulse(.004);
        secondEncoder.start();
        timer.start();
    }

    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
        getWatchdog().kill();
        timer.reset();
        timer.start();
        chassis.setSafetyEnabled(false);
        climb.setSafetyEnabled(false);
        shooter.setSafetyEnabled(false);
        loader.setSafetyEnabled(false);
        loader.set(0);
        //move(-1,-1,100);
        while (isAutonomous() && isEnabled()) {
            encoder1 = firstEncoder.get();
            encoder2 = secondEncoder.get();
            shooter.tankDrive(-1, 1);
            shootHeight();
            if(timer.get() > 4.0)
            {
                loader.set(.3);
                isLoaderEnabled=true;
            }
            /*else{
                loader.set(0);
                isLoaderEnabled=false;
            }*/
        }
        shooter.tankDrive(0, 0);
    }
    
    
    

    /*
     * Called durring autonomous to control the robot
     */
    public void move(double left, double right, double time) {
        chassis.tankDrive(left, right);
        Timer.delay(time);
        chassis.tankDrive(0.0, 0.0);
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {

        climb.setSafetyEnabled(true);
        chassis.setSafetyEnabled(true);
        loader.setSafetyEnabled(true);

        while (isOperatorControl() && isEnabled()) {
            
             if(compressor.getPressureSwitchValue()){
                compressor.start();
            } else{
                compressor.stop();
            }

            chassis.tankDrive((-1) * chassis1.getAxis(Joystick.AxisType.kY), (-1) * chassis2.getAxis(Joystick.AxisType.kY));

            limitSwitchCheck();
            cameraControl();
            encoder();
            /*
             * Shooting
             */
            shooting();
            loader();
            handleEncoderPresets();
            print();
            Timer.delay(0.01);                                                  //Pauses for .01 seconds before starting the loop again
        }//end of the while loop for OpperatorControl
    }

    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {
    }

    /*
     * 
     * Camera Controls:
     *      Button 5 for pan right
     *      Button 4 for pan left
     *      Button 3 for pan up
     *      Button 2 for pan down
     *      Trigger pan center
     *      
     *      Takes 2 Servos
     *      *Note*  Verify that each servo has a power relay
     *      Currently wired to 7 and 8
     *
     * 
     */
    public void cameraControl() {
        if (chassis2.getRawButton(3) || climbing2.getRawButton(3) && vert < cameraMaxAngle) {
            vert += .5;
            servoVertical.setAngle(vert);
        } else if (chassis2.getRawButton(2) || climbing2.getRawButton(2) && vert > cameraMinAngle) {
            vert -= .5;
            servoVertical.setAngle(vert);
        }
        if (chassis2.getRawButton(4) || climbing2.getRawButton(4) && hori > cameraMinAngle) {
            hori -= .5;
            servoHorizontal.setAngle(hori);
        } else if (chassis2.getRawButton(5) || climbing2.getRawButton(5) && hori < cameraMaxAngle) {
            hori += .5;
            servoHorizontal.setAngle(hori);
        }/*
         * if the camera gets bumped in any direction, it will automatically reset
         */
        if (servoVertical.getAngle() <= (vert - .1) || servoVertical.getAngle() >= (vert + .1)) {
            servoVertical.setAngle(vert);
        }
        if (servoHorizontal.getAngle() <= (hori - .1) || servoHorizontal.getAngle() >= (hori + .1)) {
            servoHorizontal.setAngle(hori);
        }
        /*
         * used for camera presets
         *      Hypothetically used to quickly check certain parts of the robot
         */
        if (chassis2.getRawButton(1) || climbing2.getRawButton(1)) {
            vert = 90;
            hori = 90;
        }
        if (chassis1.getRawButton(3) || climbing1.getRawButton(3) && vert < cameraMaxAngle) {
            vert = 135;
            hori = 90;
            servoVertical.setAngle(vert);
        } else if (chassis1.getRawButton(2) || climbing1.getRawButton(2) && vert > cameraMinAngle) {
            vert = 45;
            hori = 90;
            servoVertical.setAngle(vert);
        }
        if (chassis1.getRawButton(4) || climbing1.getRawButton(4) && hori > cameraMinAngle) {
            vert = 90;
            hori = 45;
            servoHorizontal.setAngle(hori);
        } else if (chassis1.getRawButton(5) || climbing1.getRawButton(5) && hori < cameraMaxAngle) {
            vert = 90;
            hori = 135;
            servoHorizontal.setAngle(hori);
        }

        /*
         * UI controls
         *      used to catch values that the user wants to save for the camera's location and direction
         */
        if (chassis1.getRawButton(8)) {
            UIHori = hori;
            UIVert = vert;
        }
        if (chassis1.getRawButton(9)) {
            hori = UIHori;
            vert = UIVert;
        }
    }

    public void limitSwitchCheck() {
        //if (limitSwitch1.get()) {
            //if (climbing2.getMagnitude() > 0) {
             //   climb.tankDrive(0, 0);
            //} else {
                climb.tankDrive(climbing1.getAxis(Joystick.AxisType.kY)*2, (-2) * climbing2.getAxis(Joystick.AxisType.kY));
                               //climb.setLeftRightMotorOutputs(climbing1.getAxis(Joystick.AxisType.kY), (-1) * climbing2.getAxis(Joystick.AxisType.kY));

            //}
        //}
        //if (limitSwitch2.get()) {
            //if (climbing1.getMagnitude() < 0) {
            //    climb.tankDrive(0, 0);
            //} else {
              //  climb.tankDrive(climbing1.getAxis(Joystick.AxisType.kY), (-1) * climbing2.getAxis(Joystick.AxisType.kY));
            //}
        //}
    }

    public void shooting() {
        if (climbing1.getRawButton(11)) {
            shooter.tankDrive(-1, 1);
        }
        else if(climbing1.getRawButton(10)){
            shooter.tankDrive(0,0);
        }
    }

    public void loader() {
        if (climbing1.getRawButton(6)) {
            solenoid.set(true);
            solenoid2.set(false);
        } else {
            solenoid.set(false);
            solenoid2.set(true);
            isLoaderEnabled=false;
        }
    }

    public void encoder() {
        /*
         * This should check to see if the user is pressing the reset button
         *  if they are then the encoder values will reset
         *  otherwise, the encoder values will update
         */
        if (chassis1.getRawButton(8) && chassis2.getRawButton(8)) {
            encoder1 = 0;
            encoder2 = 0;
            firstEncoder.free();
            secondEncoder.free();
            firstEncoder.start();
            secondEncoder.start();
        } else {
            encoder1 = firstEncoder.get();
            encoder2 = secondEncoder.get();
        }
    }

    public void print() {
        counter++;

        if (counter == 100) {       //this will prevent the print screen from getting cluttered
            clearPrint();
            counter = 0;
        }

        robot.println(DriverStationLCD.Line.kUser1, 1, "Lower(1): " + encoder1);
        robot.println(DriverStationLCD.Line.kUser2, 1, "Upper(2): " + encoder2);
        robot.println(DriverStationLCD.Line.kUser3, 1, "Loading: " + isLoaderEnabled);
        robot.println(DriverStationLCD.Line.kUser4, 1, "Shooting: "+shooter.isAlive());
        robot.println(DriverStationLCD.Line.kUser5, 1, "Timer: "+timer.get());
        robot.println(DriverStationLCD.Line.kUser6, 1, "");
        robot.updateLCD();
    }

    /*
     * By calling this function, you clear the output for the drivers station
     * This function does not require any parameters
     */
    public void clearPrint() {
        //this prints out a bunch of blank lines in order to clear the drivers station output screen
        robot.println(DriverStationLCD.Line.kUser1, 1, "                                        ");
        robot.println(DriverStationLCD.Line.kUser2, 1, "                                        ");
        robot.println(DriverStationLCD.Line.kUser3, 1, "                                        ");
        robot.println(DriverStationLCD.Line.kUser4, 1, "                                        ");
        robot.println(DriverStationLCD.Line.kUser5, 1, "                                        ");
        robot.println(DriverStationLCD.Line.kUser6, 1, "                                        ");

        robot.updateLCD(); //this updates the drivers station output screen, allowing everything to show up correctly
    }
    
    
    
    public void handleEncoderPresets() {
        if(climbing2.getRawButton(6)||chassis2.getRawButton(6)) {
            loadHeight();
        }
        else if(climbing2.getRawButton(7)||chassis2.getRawButton(7)) {
            shootHeight();
        }
        else if(climbing2.getRawButton(11)||chassis2.getRawButton(11)) {
            climbHeight();
        }
        else if((climbing1.getRawButton(8)&&climbing2.getRawButton(8)) || (chassis1.getRawButton(8)&&chassis2.getRawButton(8))) {
            adjustZero=true;
            moveToZero();
        }
        else if(climbing2.getRawButton(10)||chassis2.getRawButton(10)) {
            blockingHeight();
        }
    }
    
    
    
    public boolean moveToZero() {
        double pos1=0;
        double pos2=0;
        pos1 = calculateThrottle(encoder1, 0);
        pos2 = calculateThrottle(encoder2, 0);
        climb.tankDrive(pos1, (-1)*pos2);
        
        boolean atPosition = false;
        if (Math.abs(pos1) < 0.01 && Math.abs(pos2) < 0.01)
        {
            atPosition = true;
        }
        
        return atPosition;
    }
    
    public boolean blockingHeight() {
        double pos1=0;
        double pos2=0;
        
        pos1 = calculateThrottle(encoder1, blockingHeight1);
        pos2 = calculateThrottle(encoder2, blockingHeight2);
        
        climb.tankDrive(pos1, (-1)*pos2);
        
        boolean atPosition = false;
        if (Math.abs(pos1) < 0.01 && Math.abs(pos2) < 0.01)
        {
            atPosition = true;
        }
        
        return atPosition;
    }
    
    public boolean shootHeight() {
        climbingCounter++;
        double pos1=0;
        double pos2=0;
        
        pos1 = calculateThrottle(encoder1, shootingHeight1);
        pos2 = calculateThrottle(encoder2, shootingHeight2);
        climb.tankDrive(pos1, (-1)*pos2);
        
        boolean atPosition = false;
        if (Math.abs(pos1) < 0.01 && Math.abs(pos2) < 0.01)
        {
            atPosition = true;
        }
        
        return atPosition;
    }
    
    
    
    public boolean loadHeight() {
        double pos1=0;
        double pos2=0;
        
        pos1 = calculateThrottle(encoder1, loadingHeight1);
        pos2 = calculateThrottle(encoder2, loadingHeight2);
        
        climb.tankDrive(pos1, (-1)*pos2);
        
        boolean atPosition = false;
        if (Math.abs(pos1) < 0.01 && Math.abs(pos2) < 0.01)
        {
            atPosition = true;
        }
        
        return atPosition;
    }
    
    private double calculateThrottle(double currPos, double commandedPos)
    {
        double throttle = 0.0;
        
        double delta = Math.abs(currPos - commandedPos);
        
        if (delta > 150)
        {
            throttle = 1.0;
        }
        else if (delta > 100)
        {
            throttle = 0.5;
        }
        else if (delta > 50)
        {
            throttle = 0.275;
        }
        else
        {
            throttle = 0;
        }
        
        if (currPos > commandedPos)
        {
            throttle *= -1;
        }
        
        return throttle;
    }
    
    
    
    public void climbHeight() {
        double pos1=0;
        double pos2=0;
        
        pos1 = calculateThrottle(encoder1, climbingHeight1);
        pos2 = calculateThrottle(encoder2, climbingHeight2);
        
        climb.tankDrive(pos1, (-1)*pos2);
        
        boolean atPosition = false;
        if (Math.abs(pos1) < 0.01 && Math.abs(pos2) < 0.01)
        {
            atPosition = true;
        }
        
    }
}
/*
 * ChangeLog
 *  added in a camera position logger
 *  chassis1 button 8
 * 
 * Organized
 *  organized everything into functions
 *  this should make parts of the code much easier to find
 *  it will also make it easier for other people to understand the code
 * 
 * Encoder Info
 *  shooting position:
 *      encoder1: -18000
 *      encoder2: 0
 * 
 * down
 * servohorizontal: 90
 * servovertical: 25
 * 
 * left
 * 
 * 
 * encoder2
 * 13268
 * 
 * when joystick 4 is pushed forward it increases encoder1 positivly?
 * backwards on 4 = encoder2 negative
 * forward on 3=encoder 1 negative
 * 
 * encoder1:
 * -29229
 * 
 */

/*
 * autonomous notes
 *  have the robot move using the encoders
 *  then reset the encoders to 0
 *  then move again
 */

/*
 * cool things for next year
 *      a camera on the robot that can save the mach video
 */

/*
 * Buttons needed:
 *  Button for aiming height
 *  Button for loading height
 *  Button for climbing height
 */

/*
 * Buttons used
 *  chassis2 || climbing2   .getRawButton(3)
 *      Moves the camera UP
 *  chassis2 || climbing2   .getRawButton(2)
 *      Moves the camera DOWN
 *  chassis2 || climbing2   .getRawButton(4)
 *      Moves camera to the LEFT
 *  chassis2 || climbing2   .getRawButton(5)
 *      Moves camera to the RIGHT
 *  chassis2 || climbing2   .getRawButton(1)
 *      Resets the camera to the STARTING POSITION
 *  chassis1 || climbing1   .getRawButton(3)
 *      Moves the camera to the UP preset
 *  chassis1 || climbing1   .getRawButton(2)
 *      Moves the camera to the DOWN preset
 *  chassis1 || climbing1   .getRawButton(4)
 *      Moves the camera to the LEFT preset
 *  chassis1 || climbing1   .getRawButton(5)
 *      Moves the camera to the RIGHT preset
 *  climbing1.getRawButton(11)
 *      turnes the shooter ON
 *  climbing1.getRawButton(10)
 *      turnes the shooter OFF
 *  climbing1.getRawButton(6)
 *      turnes the loader ON only while this button is being pressed
 *  chassis1 && chassis2   .getRawButton(8)
 *  
 *  
 * 
 */

//4 3 5
//  2

//loading       controler4 button6
//shooting      controler4 button7
//climbing      controler4 button11


//add things to the print function that will tell if the loader and the shooter are spinning

//-4000
//-5000