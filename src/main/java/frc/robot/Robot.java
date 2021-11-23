/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class Robot extends TimedRobot {

    //Initiate PID controller
    private CANPIDController m_pidController;
    //Needed to adjust numbers in PID when testing
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    

    //initiate the Joystick
    //Not needed when transferred to main code
    public static Joystick joystick0 = new Joystick(1); 
    //Omit when FMS is in place
    public static XboxController xboxCTR = new XboxController(0); //initate xbox controller

    //When transferred to main code
    //Declare this in constants
    public int colorMotorDeviceId = 5;

    //initiate the motor (stays in here)
    public CANSparkMax colorMotor = new CANSparkMax (colorMotorDeviceId, MotorType.kBrushless); 
    CANEncoder encoderSensor = colorMotor.getEncoder(); //intiate the enocder code
    CANError brake = colorMotor.setIdleMode ( IdleMode.kBrake ); //brake mode for the spark max
    public CANPIDController motorController = colorMotor.getPIDController();
    //initiate the color sensor
    private final I2C.Port i2cPort = I2C.Port.kOnboard;  
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort); 

    //initializes colorMatcher
    private final ColorMatch m_colorMatcher = new ColorMatch();
    //set the color parameters for the color sensor
    private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    @Override

    public void robotInit() {
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);

        //Resets the color motor
        encoderSensor.setPosition(0.0);
        colorMotor.restoreFactoryDefaults();

        // initialze PID controller and encoder objects
        m_pidController = colorMotor.getPIDController();
        encoderSensor = colorMotor.getEncoder();

        // PID coefficients
        kP = 0.000050; 
        kI = 0.000001;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000156; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;
        minVel = 0;
        allowedErr = 2;
        // Smart Motion Coefficients
        maxVel = 200; // rpm
        maxAcc = 150;

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
        int smartMotionSlot = 0;
        m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
        SmartDashboard.putBoolean("Mode", true);
    }

    //Fields
    double amountOfRevolutions = 0;
    int stopCounter = 0, colorChange = 0; 
    //ButtonPress = true turns revolutionButton and positionButton false
    boolean revolutionButton = false, positionButton = false, isInPIDMode = false, buttonPress = false;
    boolean buttonGreen = false , buttonRed = false, buttonYellow = false, buttonBlue = false;
    double setSpeed = 0.085;
    int PIDCounter = 0;
    double manualControl = 0;
    int targetColor = 0;
    @Override

    public void robotPeriodic() {
        SmartDashboard.putNumber("Set Position", 0);
        //start of color thing
        Color detectedColor = m_colorSensor.getColor();
        double amountOfRotations = encoderSensor.getPosition();
        double ratePerMinuteForTheMotor = encoderSensor.getVelocity();

        int currentColor = 0;
        String colorString;
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == kBlueTarget) { //Sets the variable to the current color the motor is on. Will change as wheel spins
            colorString = "Blue";
            currentColor = 1;
        } else if (match.color == kRedTarget) {
            colorString = "Red";
            currentColor = 2;
        } else if (match.color == kGreenTarget) {
            colorString = "Green";
            currentColor = 3;
        } else if (match.color == kYellowTarget) {
            colorString = "Yellow";
            currentColor = 4;
        } else {
            colorString = "Unknown";
        }

        //set the buttons for the joystick
        if (joystick0.getRawButtonPressed(11) == true){
            revolutionButton = true;
            SmartDashboard.putBoolean("Mode", true); //starts the revolution control
        }else if (xboxCTR.getBumperPressed(Hand.kRight) == true){
            positionButton = true;
            SmartDashboard.putBoolean("Mode", false); //starts the position control
        }else if (positionButton == true){
            if (xboxCTR.getBButtonPressed() == true){
                buttonRed = true; //if both position control and color bottons are pressed, the motor will spin until the sensor finds that color
                PIDCounter = 0;
            } if (xboxCTR.getAButtonPressed() == true){
                buttonGreen = true;
                PIDCounter = 0;
            } if ( xboxCTR.getXButtonPressed() == true){
                buttonBlue = true;
                PIDCounter = 0;
            } if ( xboxCTR.getYButtonPressed() == true){
                buttonYellow = true;
                PIDCounter = 0;
            }//WE HAVE TO CUT DOWN ON THE BUTONS SO WE HAVE TO COMBINE THE POSITION AND ROTATION CONTROL INTO ONE SEQUENCE, AND THE OPERATOR CAN MOVE OFF ONCE THE STAGE IS ACTIVATED
            //WE ALSO NEED TO MAKE A BUTTON FOR THE MANUAL CONTROL
        }else if(positionButton == false){
            buttonRed = false;
            buttonGreen = false;
            buttonBlue = false;
            buttonYellow = false;
        }

        //Control panel logic beginning
        //The operator presses a button that initites position control(position control)

        double setPoint, processVariable;
        boolean mode = SmartDashboard.getBoolean("Mode", false);

        if(mode == true) {
            processVariable = encoderSensor.getVelocity();
            //Start of amount of revolutions
            //Can use encoder to count but you need to consider wheel size
            if (revolutionButton == true) { //Starts the mode for revolution control
                if (stopCounter == 0){
                    //Set the color it starts on, is not subject to change
                    if (match.color == kBlueTarget){
                        colorChange = 1;
                    } else if (match.color == kRedTarget){
                        colorChange = 2;
                    }else if (match.color == kGreenTarget){
                        colorChange =3;
                    } else if (match.color == kYellowTarget){
                        colorChange = 4;
                    }
                    stopCounter++;
                }
                colorMotor.set(.075);
                if (currentColor != colorChange){ //Starts counting the amount of times the color changes
                    amountOfRevolutions += 1; 
                    colorChange = currentColor;
                }
                if (amountOfRevolutions >= 32){ //This stops the motor from turning the color wheel
                    colorMotor.set(0);
                    revolutionButton = false;
                    if (revolutionButton == false){
                        amountOfRevolutions = 0;
                    }
                }
            }
            // end of amount of revoultions

        } else {
            processVariable = encoderSensor.getPosition();
            //Start of position control
            if (positionButton == true){  
                if (positionButton){
                    if(buttonBlue){
                    if (currentColor == 1){
                        if (PIDCounter == 0){
                            colorMotor.set(0);
                            encoderSensor.setPosition(0);
                            PIDCounter = 1;
                        }
                        setPoint = SmartDashboard.getNumber("Set Position", -0.5);
                        m_pidController.setReference(setPoint, ControlType.kSmartMotion);
                        //When the motor stops moving click button 10 on joystick to stop PID
                    }else if (currentColor != 1){
                        PIDCounter = 0;
                        colorMotor.set(0.065);
                    }
                  }
                    if(buttonRed){
                    if (currentColor == 2){
                        if (PIDCounter == 0){
                            colorMotor.set(0);
                            encoderSensor.setPosition(0);
                            PIDCounter = 1;
                        }
                        setPoint = SmartDashboard.getNumber("Set Position", -0.5);
                        m_pidController.setReference(setPoint, ControlType.kSmartMotion);

                    }else if (currentColor != 2){
                        PIDCounter = 0;
                        colorMotor.set(0.065);
                    }
                  }
                    if(buttonGreen){
                    if (currentColor == 3){
                        if (PIDCounter == 0){
                            colorMotor.set(0);
                            encoderSensor.setPosition(0);
                            PIDCounter = 1;
                        }
                        setPoint = SmartDashboard.getNumber("Set Position", -0.5);
                        m_pidController.setReference(setPoint, ControlType.kSmartMotion);

                    }else if (currentColor != 3){
                        PIDCounter = 0;
                        colorMotor.set(0.065);
                    }
                  }
                    if(buttonYellow){
                    if (currentColor == 4){
                        if (PIDCounter == 0){
                            colorMotor.set(0);
                            encoderSensor.setPosition(0);
                            PIDCounter = 1;
                        }
                        setPoint = SmartDashboard.getNumber("Set Position", -0.5);
                        m_pidController.setReference(setPoint, ControlType.kSmartMotion);

                    }else if (currentColor != 4){
                        PIDCounter = 0;
                        colorMotor.set(0.065);
                    }
                }
              }
            }
        }//manual override for manualness
        //Manual control over the color wheel
        if (joystick0.getRawButtonPressed(10) == true){
            buttonPress = true;
        }else if (joystick0.getRawButtonReleased(10) == true){
            buttonPress = false;
            colorMotor.set(0);
        }
        if(buttonPress == true){
            colorMotor.set(0.1);
            positionButton = false;
            revolutionButton = false;
            // joystick0.getPOV();
        }

        //end control panel logic
        //NEED PROXIMITY SENSOR ONLY IF THE PERSON WONT BE LOOKING AT A SENSOR
        //smartdashboard SmartDashboard.putNumber("Speed of Motor", ratePerMinuteForTheMotor);
        SmartDashboard.putNumber("Encoder Clicks", amountOfRotations);
        SmartDashboard.putNumber("Amount of Revolutions", amountOfRevolutions);
        SmartDashboard.putNumber("Confidence", match.confidence);
        SmartDashboard.putString("Detected Color", colorString);
        SmartDashboard.putNumber("Process Variable", processVariable);
    }
}







