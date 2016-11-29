package parkingRobot.hsamr0;


import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IControl.ControlMode;
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
	
	/**
	 * line information measured by left light sensor: in value
	 */
	
	NXTMotor leftMotor = null;
	NXTMotor rightMotor = null;
	
	IPerception perception = null;
	INavigation navigation = null;
	IMonitor monitor = null;
	ControlThread ctrlThread = null;
	

    int leftMotorPower = 0;
	int rightMotorPower = 0;
	
	double velocity = 0.0; //v
	double angularVelocity = 0.0; //w
	
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
    boolean kurve_alt =false;
    int ecke =0;
    int eckengezählt=0;
    int f =1;
    double dist_gerade = 0;   /* Distanz für Geradeausfahrt cm*/
    double dist_kurve = 360; /* Drehung in °*/
	double pidL=0;
	double pidR=0;
	int i =0;
    int j =0;
    private int speedL =0;

	private int speedR = 0;
	
	int vsoll = 0;
	int wsoll = 0;
	
	double distance_old = 0;
	
	double leftAnglesum = 0;
	double rightAnglesum =0;
	
	double driving_direction = 1;

	
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
		
		//this.UOdmometry                 = perception.getUOdmometryDiffernce()
				;
		
		// MONITOR (example)
		monitor.addControlVar("RightSensor");
		monitor.addControlVar("LeftSensor");
		
		//monitor.addControlVar("LeftAnglem");
		//monitor.addControlVar("RightAnglem");
		
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
		this.encoderLeft  = perception.getControlLeftEncoder();
		this.encoderRight = perception.getControlRightEncoder();
		
		this.angleMeasurementLeft  	= this.encoderLeft.getEncoderMeasurement();

		this.angleMeasurementRight 	= this.encoderRight.getEncoderMeasurement();

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
			
		if (kurve_alt = true && !kurve){
			
			ecke ++;
			eckengezählt= ecke % 7;
			
		}
		
		kurve_alt= false;
		
		
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
			kurve_alt = true;
		}
		
		if (i>=5){
			pidL=30;
			pidR=0;
			kurve_alt = true;
		}
		
		
		
			
			
		rightMotor.setPower((int)(highPower-Math.round(pidL))); // Runden auf int durch Math.round
		leftMotor.setPower((int)(highPower-Math.round(pidR)));
		
		
	}
		
		
	public int getEcke(){
		return eckengezählt;
	}

	
	
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
	/*private void drive(double v, double omega){
	    leftMotor.forward();
		rightMotor.forward();
		double n= 0.5; //?
		v=30;
		v=n*v;
		double distance = 10;
		
		
		this.angleMeasurementLeft  	= this.encoderLeft.getEncoderMeasurement();

		this.angleMeasurementRight 	= this.encoderRight.getEncoderMeasurement();
		
		double encoderLeft = new IPerception.EncoderSensor();

		if(omega == 0 ){
		while (distance != distLeft){
		distLeft = ((this.encoderLeft) * distance)/ (Math.PI * 0.56)); //distance of left  wheel in cm/s
		rightMotor.setPower((int)(v)); 
		leftMotor.setPower((int)(v));
			
		}
			
		this.stop();	
		this.angleMeasurementLeft =0; 
		distLeft = 0;
		}
		
		if (omega !=0){
		}
		//double w 			= (vRight - vLeft) / 125; //angular velocity of robot in rad/s

		rightMotor.setPower((int)(v)); 
		leftMotor.setPower((int)(v));
		
		
	}
	*/
	
	


	/** the distance between the wheels in m */

	private static final double WHEEL_DISTANCE = 18.2; /*12cm Radabstand + 6 cm Offset durch empirisches probieren*/

	/** the radius of the wheels */

	private static final double WHEEL_RADIUS = 5.5; 

	/** the distance traveled when the wheel turns one rad */

	private static final double DISTANCE_PER_ROTATION = WHEEL_RADIUS * Math.PI    ;
	
	  
	


	/** constructs a new VWCommand 

	 * @param v			The desired velocity of the robots center

	 * @param w			The desired angular velocity of the robot around its center
	 

	 */
	
	
	

	public void drive (double v, double w){
		
		
		
		
		//driving_direction = 1;
		
		if ( driving_direction   == 1){
			
			leftMotor.forward();
			rightMotor.forward();
			
			
			}
			
			if ( driving_direction == 0){
				
				leftMotor.backward();
				rightMotor.backward();
				
				}
		
		leftMotor.forward();
		rightMotor.forward();
	
		//Beispielprogramm:
		 
		  switch (f){
		  
		  case 1: 
		  dist_gerade = 150;
		  dist_kurve = 0;
		  vsoll = 10;
		  wsoll = 0;
		  break;
		  case 2:
			  
		  dist_gerade = 0;
		  dist_kurve = 90;
		  vsoll = 0;
		  wsoll= 30;
		  break;
		  
		  case 3:
		  dist_gerade = 30;
		  dist_kurve = 0;
		  vsoll = 5;
		  wsoll = 0;
		  break;
		  
		  case 4:
		  dist_gerade = 0;
		  dist_kurve = 90;
		  vsoll = 0;
		  wsoll= 30;
		  break;
		    
		 case 5:
		 setCtrlMode(ControlMode.LINE_CTRL);
		 break;
		  
		  }
		  monitor.writeControlComment("Zustandf " + f);
		
		
		dist_gerade = dist_gerade /DISTANCE_PER_ROTATION; 
		
		
		dist_kurve = (dist_kurve /90) * (WHEEL_DISTANCE/ 2)/ DISTANCE_PER_ROTATION ; /* notwendige Radumdrehungen*/
		
		double leftAngle 	= this.angleMeasurementLeft.getAngleSum();
		double rightAngle 	= this.angleMeasurementRight.getAngleSum();
		Pose winkel       = navigation.getPose();
		//double winkelresult = this.winkel
		
		
		
		leftAnglesum = leftAnglesum + leftAngle; 
		rightAnglesum = rightAnglesum + rightAngle;
		
		monitor.writeControlComment("vsol " + vsoll);
		
		monitor.writeControlComment("wsol " + wsoll);	
		monitor.writeControlComment("RightAngle "+ rightAnglesum); 
		monitor.writeControlComment("left Angle " + leftAnglesum);
		
		//vsoll = 0; // Geschwindigkeit in cm/s 
		//wsoll = 90; // Drehung in °/s
		
	//EncoderSensor test = controlRightEncoder;
		//distance_old = this.UOdmometry.getUOdmometryDiffernce();
	
	v= vsoll * 3 * DISTANCE_PER_ROTATION ; // 3 entspricht 1 cm/s 
	w=  wsoll* 0.9;

	

		if (w == 0){
			
			
				{if (Math.abs(dist_gerade) > leftAnglesum /360 ){

			    speedL = speedR = (int) (v / DISTANCE_PER_ROTATION );
			    

						} 
				else {
					this.stop();
					f++;
					leftAnglesum = 0;
					rightAnglesum = 0;

					}
				}
		}

		else if (v == 0 && w> Math.abs(45))

		{ 
			{if (Math.abs(dist_kurve) >  Math.abs(leftAnglesum /360)){

			   speedL = (int) (-WHEEL_DISTANCE / 2.0 * w / DISTANCE_PER_ROTATION);

			   speedR = (int) (WHEEL_DISTANCE / 2.0 * w / DISTANCE_PER_ROTATION);

					}
				else {
					this.stop();
					f++;
					leftAnglesum = 0;
					rightAnglesum = 0;
					}

			

			}
		}
				
		
		else if (v == 0 && w<= 45 && w >0)

		{ 
			{if (2 * Math.abs(dist_kurve) > Math.abs(rightAnglesum /360 )){

			speedR = (int) (WHEEL_DISTANCE * w / DISTANCE_PER_ROTATION);
			speedL =0;

			}
			else {
				this.stop();
				f++;
				leftAnglesum = 0;
				rightAnglesum = 0;
				}

			}
		}
		
		else if (v == 0 && w<= - 45 && w >0)
			

		{
			if (2 * Math.abs(dist_kurve) > Math.abs(leftAnglesum /360 )){

			speedL = (int) (WHEEL_DISTANCE * w / DISTANCE_PER_ROTATION);
			speedR = 0;

			}
			else {
				this.stop();
				f++;
				leftAnglesum = 0;
				rightAnglesum = 0;
				}
			
			
			}
		else if (v == 0 && w ==0){
			speedR=0;
			speedL= 0;
		}


	/*	else

		{

			double radius = v / w;

			speedL = (int) ((radius - WHEEL_DISTANCE / 2.0) * v / radius / DISTANCE_PER_ROTATION);

			speedR = (int) ((radius + WHEEL_DISTANCE / 2.0) * v / radius / DISTANCE_PER_ROTATION);

		}
		
     */
		 
		
		
		rightMotor.setPower(speedR); 
		
		leftMotor.setPower(speedL);
				}
				
		
	public void prepare(int speedLeft, int speedRight, Pose currentPose) {

	}
}	
		
	
	
