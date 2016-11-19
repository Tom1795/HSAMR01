package parkingRobot.hsamr0;


import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;
import lejos.nxt.NXTMotor;
import parkingRobot.INavigation;


/**
 * Main class for control module
 *
 */
public class ControlRST implements IControl {
	
	/**
	 * reference to {@link IPerception.EncoderSensor} class for left robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderLeft							=	null;
	/**
	 * reference to {@link IPerception.EncoderSensor} class for right robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderRight							=	null;
	
	/**
	 * reference to data class for measurement of the left wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementLeft 	= 	null;
	/**
	 * reference to data class for measurement of the right wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementRight	= 	null;
	
	/**
	 * line information measured by right light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorRight	=	0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorLeft	=	0;
	/**
	 * line information measured by left light sensor: in percent
	 */
	double lineSensorRightValue	=	0;
	double lineSensorLeftValue	=	0;
	
	NXTMotor leftMotor = null;
	NXTMotor rightMotor = null;
	
	IPerception perception = null;
	INavigation navigation = null;
	IMonitor monitor = null;
	ControlThread ctrlThread = null;

    int leftMotorPower = 0;
	int rightMotorPower = 0;
	
	double velocity = 0.0;
	double angularVelocity = 0.0;
	
	Pose startPosition = new Pose();
	Pose currentPosition = new Pose();
	Pose destination = new Pose();
	
	ControlMode currentCTRLMODE = null;
	
	EncoderSensor controlRightEncoder    = null;
	EncoderSensor controlLeftEncoder     = null;

	int lastTime = 0;
	
    double currentDistance = 0.0;
    double Distance = 0.0;
    boolean merkerr=false;
    boolean merkerl=false;
    boolean kurve=false;
	double pidL=0;
	double pidR=0;
	int i =0;
    int j =0;

	
	/**
	 * provides the reference transfer so that the class knows its corresponding navigation object (to obtain the current 
	 * position of the car from) and starts the control thread.
	 * 
	 * @param perception corresponding main module Perception class object
	 * @param navigation corresponding main module Navigation class object
	 * @param monitor corresponding main module Monitor class object
	 * @param leftMotor corresponding NXTMotor object
	 * @param rightMotor corresponding NXTMotor object
	 */
	public ControlRST(IPerception perception, INavigation navigation, NXTMotor leftMotor, NXTMotor rightMotor, IMonitor monitor){
		this.perception = perception;
        this.navigation = navigation;
        this.monitor = monitor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		this.currentCTRLMODE = ControlMode.INACTIVE;
			
		this.encoderLeft  = perception.getControlLeftEncoder();
		this.encoderRight = perception.getControlRightEncoder();
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
		this.lineSensorRightValue		= perception.getRightLineSensorValue();
		this.lineSensorLeftValue  		= perception.getLeftLineSensorValue();
		
		// MONITOR (example)
		monitor.addControlVar("RightSensor");
		monitor.addControlVar("LeftSensor");
		
		this.ctrlThread = new ControlThread(this);
		
		ctrlThread.setPriority(Thread.MAX_PRIORITY - 1);
		ctrlThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate
		ctrlThread.start();
	}
	

	// Inputs
	
	/**
	 * set velocity
	 * @see parkingRobot.IControl#setVelocity(double velocity)
	 */
	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}

	/**
	 * set angular velocity
	 * @see parkingRobot.IControl#setAngularVelocity(double angularVelocity)
	 */
	public void setAngularVelocity(double angularVelocity) {
		this.angularVelocity = angularVelocity;

	}
	
	/**
	 * set destination
	 * @see parkingRobot.IControl#setDestination(double heading, double x, double y)
	 */
	public void setDestination(double heading, double x, double y){
		this.destination.setHeading((float) heading);
		this.destination.setLocation((float) x, (float) y);
	}
	

	
	/**
	 * sets current pose
	 * @see parkingRobot.IControl#setPose(Pose currentPosition)
	 */
	public void setPose(Pose currentPosition) {
		// TODO Auto-generated method stub
		this.currentPosition = currentPosition;
	}
	

	/**
	 * set control mode
	 */
	public void setCtrlMode(ControlMode ctrl_mode) {
		this.currentCTRLMODE = ctrl_mode;
	}
		
	/**
	 * set start time
	 */
	public void setStartTime(int startTime){
		this.lastTime = startTime;
	}
	
	/**
	 * selection of control-mode
	 * @see parkingRobot.IControl#exec_CTRL_ALGO()
	 */
	public void exec_CTRL_ALGO(){
		
		switch (currentCTRLMODE)
		{
		  case LINE_CTRL	: update_LINECTRL_Parameter();
		                      exec_LINECTRL_ALGO();
		                      break;
		  case VW_CTRL		: update_VWCTRL_Parameter();
		   					  exec_VWCTRL_ALGO();
		   					  break; 
		  case SETPOSE      : update_SETPOSE_Parameter();
			  				  exec_SETPOSE_ALGO();
		                      break;
		  case PARK_CTRL	: update_PARKCTRL_Parameter();
		  					  exec_PARKCTRL_ALGO();
		  					  break;		  					  
		  case INACTIVE 	: exec_INACTIVE();
			                  break;
		}

	}
	
	// Private methods
	
	/**
	 * update parameters during VW Control Mode
	 */
	private void update_VWCTRL_Parameter(){
		setPose(navigation.getPose());
	}
	
	/**
	 * update parameters during SETPOSE Control Mode
	 */
	private void update_SETPOSE_Parameter(){
		setPose(navigation.getPose());
	}
	
	/**
	 * update parameters during PARKING Control Mode
	 */
	private void update_PARKCTRL_Parameter(){
		//Aufgabe 3.4
	}

	/**
	 * update parameters during LINE Control Mode
	 */
	private void update_LINECTRL_Parameter(){
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();	
		this.lineSensorRightValue		= perception.getRightLineSensorValue();
		this.lineSensorLeftValue  		= perception.getLeftLineSensorValue();
	}
	
	/**
	 * The car can be driven with velocity in m/s or angular velocity in grade during VW Control Mode
	 * optionally one of them could be set to zero for simple test.
	 */
    private void exec_VWCTRL_ALGO(){  
		this.drive(this.velocity, this.angularVelocity);
	}
	
    private void exec_SETPOSE_ALGO(){
    	//Aufgabe 3.3
	}
	
	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO(){
		//Aufgabe 3.4
	}
	
    private void exec_INACTIVE(){
    	this.stop();
	}
	
	/**
	 * DRIVING along black line
	 * Minimalbeispiel
	 * Linienverfolgung fuer gegebene Werte 0,1,2
	 * white = 0, black = 2, grey = 1
	 */
    
/*	private void exec_LINECTRL_ALGO(){
		leftMotor.forward();
		rightMotor.forward();
		int lowPower = 1;
		int highPower = 30;
		
		// MONITOR (example)
		monitor.writeControlVar("LeftSensor", "" + this.lineSensorLeft);
		monitor.writeControlVar("RightSensor", "" + this.lineSensorRight);

        if(this.lineSensorLeft == 2 && (this.lineSensorRight == 1)){
			
			// when left sensor is on the line, turn left
    	    leftMotor.setPower(lowPower);
			rightMotor.setPower(highPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn left");
			
		} 
        else if(this.lineSensorRight == 2 && (this.lineSensorLeft == 1)){
		
			// when right sensor is on the line, turn right
			leftMotor.setPower(highPower);
			rightMotor.setPower(lowPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn right");
		}
		else if(this.lineSensorLeft == 2 && (this.lineSensorRight == 0)){
			
			// when left sensor is on the line, turn left
			leftMotor.setPower(lowPower);
			rightMotor.setPower(highPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn left");
			
		} 
		else if(this.lineSensorRight == 2 && (this.lineSensorLeft == 0)){
		
			// when right sensor is on the line, turn right
			leftMotor.setPower(highPower);
			rightMotor.setPower(lowPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn right");
		}
		else if(this.lineSensorLeft == 1 && this.lineSensorRight == 0) {
				
			// when left sensor is on the line, turn left
			leftMotor.setPower(lowPower);
			rightMotor.setPower(highPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn left");
				
		} 
		else if(this.lineSensorRight == 1 && this.lineSensorLeft == 0) {
			
			// when right sensor is on the line, turn right
			leftMotor.setPower(highPower);
			rightMotor.setPower(lowPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn right");
		}
	}*/
	
	private void exec_LINECTRL_ALGO(){
		leftMotor.forward();
		rightMotor.forward();
		//int lowPower = 1;
		int highPower = 30;
		double Kp = 17;
		double Ki = 0;
		int Kd = 0;
		double integralL = 0;
		double integralR = 0;
		double fehleraltL = 0;
		double fehleraltR = 0;
		double derivateL = 0;
		double derivateR = 0;
	

		double errorL=0;
		double errorR=0;
		
		
		
		if (!kurve){
	  // MONITOR (example)
			monitor.writeControlVar("LeftSensor", ""+this.lineSensorLeftValue);	
			monitor.writeControlVar("RightSensor", ""+this.lineSensorRightValue);	
			
		errorL = (100- lineSensorLeftValue)/100;
		errorR = (100 -lineSensorRightValue)/100;
		
		integralL = integralL + errorL;
		integralR = integralR + errorR;
		
		derivateL = errorL - fehleraltL;
		derivateR = errorR - fehleraltR;
		
		pidR = (errorL * Kp) + ( integralL * Ki) + ( derivateL * Kd);
		pidL = (errorR * Kp) + ( integralR * Ki) + ( derivateR * Kd);
		
		}
		

		
		fehleraltL = errorL;
		fehleraltR = errorR;
		
		if((this.lineSensorLeft != 2) && (this.lineSensorRight == 2)){ 
			i++;
			j=0;}
		if((this.lineSensorLeft == 2) && (this.lineSensorRight != 2)){ 
			j++;
			i=0;}
			
		if (j>=5){
			pidL=0;
			pidR=30;
		}
		
		if (i>=5){
			pidL=30;
			pidR=0;
		}
			
			
			/*if(merkerr){
				pidL=0;
				pidR=30;
			}
			if(merkerl){
				pidL=30;
				pidR=0;
			}
			kurve=true;
			merkerr=false;
			merkerl=false;
		}
		if((this.lineSensorLeft == 2) && (this.lineSensorRight != 2)){ 
			merkerl=true;
			merkerr=false;
		}
		if((this.lineSensorLeft != 2) && (this.lineSensorRight == 2)){ 
			merkerl=false;
			merkerr=true;
		}
		*/
		rightMotor.setPower((int)(highPower-Math.round(pidL))); // Runden auf int durch Math.round
		leftMotor.setPower((int)(highPower-Math.round(pidR)));
		
	}
		
		
		
			
	
		
		// else if (this.lineSensorLeft == 2 && (this.lineSensorRight ==2)){
		
			/* boolean merkerL = false;
			 * boolean merkerR = false;
			 * if(this.lineSensorLeft == 2 && (this.lineSensorRight == 0)){
			 * merkerL = true;
			 * merkerR = false;
			 * }
			 * 
			 * else if(this.lineSensorLeft == 0 && (this.lineSensorRight == 2)){
			 * merkerL = false;
			 * merkerR = true;
			 * }
			 * 
			 * 
			 */
		
	
		
		
	/*	private void exec_LINECTRL_ALGO(){
		leftMotor.forward();
		rightMotor.forward();
		int lowPower = 1;
		int highPower = 30;
		float Kp =10;

	while(1){
		
		float errorL = (100- leftSensorValue)/100;
		float errorR = (100- rightSensorValue)/100;
		
		float pidL = errorL * Kp;
		float pidR = errorR * Kp;
		
		leftMotor.setPower(highPower-Math.round(pidL)); // Runden auf int durch Math.round
		rightMotor.setPower(highPower-Math.round(pidR));

	
	}
	
	*/
	
	
	private void stop(){
		this.leftMotor.stop();
		this.rightMotor.stop();
	}
		
    /**
     * calculates the left and right angle speed of the both motors with given velocity 
     * and angle velocity of the robot
     * @param v velocity of the robot
     * @param omega angle velocity of the robot
     */
	private void drive(double v, double omega){
		//Aufgabe 3.2
	}
}