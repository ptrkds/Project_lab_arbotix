#include <math.h>

#define Arbotix
#ifdef Arbotix
  #include <Commander.h>
  #include <ax12.h>
  #include <BioloidController.h>
#endif

// Length in mm
#define L1 82.0
#define L2 101.0
#define L3 101.0
#define L4 101.0

//#define Center_req
#define X_POS_REQ
#define Y_POS_REQ
#define Z_POS_REQ
#define PHI_REQ
#define PINCHER_REQ
#define LEFT_HANDED
//#define DEBUG

// Create bioloid object for the controller
#ifdef Arbotix
  BioloidController bioloid = BioloidController(1000000);
#endif



void setup() {
  
  // Open serial connection
  Serial.begin(9600);
  delay (500);   
  Serial.println("###########################");    
  Serial.println("Serial Communication Established.");  

  // If 'set all servo to center position' is required
  // Set everything to center
  Serial.println("###########################");
  
  //TODO interpolation to center pos
  Serial.println("Set all servo motor to center (512).");
  #ifdef Arbotix
 
    const PROGMEM unsigned int m_init_pos[] = {5, 512, 512, 512, 512, 512};
    set_position_slowly(m_init_pos);
    
	delay(1000);
	
    #ifdef CENTER_REQ
      for (int i = 1; i<= 5; i++){
        Relax(i);  
      }
	#endif
	
  #endif
}

void loop() {

  // Variable init
  int m_x_pos=0;
  int m_y_pos=0;
  int m_z_pos=0;
  double m_phi=0;
  int m_pincher_pos=0;
  
  char incomingByte;
  int negative = 0;

  // If X position is required
  #ifdef X_POS_REQ

    // Read the required X value
    Serial.println("###########################");
    Serial.print("X position: "); 
  
    // Wait til the serial input
    while(!Serial.available()){}
    delay(100);
    
    // Something came across serial
    if (Serial.available() > 0) {   
      // Read the required x position
      while(Serial.available() > 0) {
        incomingByte = Serial.read();
    
        if(incomingByte == '-'){
          negative = 1;
        }else{
          // Shift left 1 decimal place
          m_x_pos *= 10;  
        
          // Convert ASCII to integer, add, and shift left 1 decimal place
          m_x_pos = ( (incomingByte - '0') + m_x_pos);
        }
      }
  
      if(negative){
        // Set X position as a negative value
        m_x_pos = 0 - m_x_pos;
  
        // Init negative again
        negative = 0;
      }
  
      Serial.println(m_x_pos);
    }
    
  #else

    m_x_pos = 0;
    
  #endif

  // If Y position is required
  #ifdef Y_POS_REQ

    // Read the required Y value
    Serial.println("###########################");
    Serial.print("Y position: "); 
  
    // Wait til the serial input
    while(!Serial.available()){}
    delay(100);
    
    // Something came across serial
    if (Serial.available() > 0) {   
      // Read the required x position
      while(Serial.available() > 0) {
        incomingByte = Serial.read();
    
        if(incomingByte == '-'){
          negative = 1;
        }else{
          // Shift left 1 decimal place
          m_y_pos *= 10;  
        
          // Convert ASCII to integer, add, and shift left 1 decimal place
          m_y_pos = ( (incomingByte - '0') + m_y_pos);
        }
      }
  
      if(negative){
        // Set Y position as a negative value
        m_y_pos = 0 - m_y_pos;
  
        // Init negative again
        negative = 0;
      }
  
      Serial.println(m_y_pos);
    }
  #else

    m_y_pos = 0;
    
  #endif

  // If Z position is required
  #ifdef Z_POS_REQ

    // Read the required Z value
    Serial.println("###########################");
    Serial.print("Z position: "); 
  
    // Wait til the serial input
    while(!Serial.available()){}
    delay(100);
    
    // Something came across serial
    if (Serial.available() > 0) {   
      // Read the required Z position
      while(Serial.available() > 0) {
        incomingByte = Serial.read();
    
        if(incomingByte == '-'){
          negative = 1;
        }else{
          m_z_pos *= 10;  // shift left 1 decimal place
        
          // Convert ASCII to integer, add, and shift left 1 decimal place
          m_z_pos = ( (incomingByte - '0') + m_z_pos);
        }
      }
  
      if(negative){
        // Set X position as a negative value
        m_z_pos = 0 - m_z_pos;
  
        // Init negative again
        negative = 0;
      }
  
      Serial.println(m_z_pos);
    }
  #else

    m_z_pos = 385;
    
  #endif

  // If Phi degree is required
  #ifdef PHI_REQ

    // Read the required Phi value
    Serial.println("###########################");
    Serial.print("Phi degree: "); 
  
    // Wait til the serial input
    while(!Serial.available()){}
    delay(100);
    
    // Something came across serial
    if (Serial.available() > 0) {   
      // Read the required phi
      while(Serial.available() > 0) {
        incomingByte = Serial.read();

        if(incomingByte == '-'){
          negative = 1;
        }else{
          m_phi *= 10;  // shift left 1 decimal place
        
          // Convert ASCII to integer, add, and shift left 1 decimal place
          m_phi = ( (incomingByte - '0') + m_phi);
        }
      }

      if(negative){
        // Set phi as a negative value
        m_phi = 0 - m_phi;
  
        // Init negative again
        negative = 0;
      }
	  
	  m_phi = m_phi * (M_PI / 180.0);
  
      Serial.println(m_phi);
    }
  #else

    m_phi = M_PI / 2.0;
    
  #endif
  
  // If Phi degree is required
  #ifdef PINCHER_REQ

    // Read the required pincher value
    Serial.println("###########################");
    Serial.print("Pincher (0/1): "); 
  
    // Wait til the serial input
    while(!Serial.available()){}
    delay(100);
    
    // Something came across serial
    if (Serial.available() > 0) {   
      // Read the required pincher position
      while(Serial.available() > 0) {
        incomingByte = Serial.read();
		
		m_pincher_pos = incomingByte - '0';
		
      }
  
      Serial.println(m_pincher_pos);
	  
	  // Open pincher
	  if(m_pincher_pos)
	    m_pincher_pos = 512;
	
    }
  #else

    m_pincher_pos = 0;
    
  #endif

  // Serial output: " (x,y,z) phi rad "
  Serial.println("###########################");
  Serial.print("Position: (");
  Serial.print(m_x_pos);
  Serial.print(",");
  Serial.print(m_y_pos);
  Serial.print(",");
  Serial.print(m_z_pos);
  Serial.print(") ");
  Serial.print(m_phi);
  Serial.print(" rad | Pincher: ");
  Serial.print(m_pincher_pos);
  Serial.println();
  Serial.println("###########################");

  // Teta calculation
  double m_teta1 = calc_teta1(m_x_pos, m_y_pos);
  double m_teta3 = calc_teta3(m_x_pos, m_y_pos, m_z_pos, m_teta1, m_phi);
  double m_teta2 = calc_teta2(m_z_pos, m_phi, m_teta3);
  double m_teta4 = calc_teta4(m_phi, m_teta2, m_teta3);

  #ifdef DEBUG
    Serial.print("Teta1: ");
    Serial.println(m_teta1);
    Serial.print("Teta2: ");
    Serial.println(m_teta2);
    Serial.print("Teta3: ");
    Serial.println(m_teta3);
    Serial.print("Teta4: ");
    Serial.println(m_teta4);
  #endif
  
  // Teta -> Position
  int m_pos1 = calc_position_from_teta(m_teta1);
  int m_pos2 = calc_position_from_teta(m_teta2);
  int m_pos3 = calc_position_from_teta(m_teta3);
  int m_pos4 = calc_position_from_teta(m_teta4);
  
  Serial.print("Pos1: ");
  Serial.println(m_pos1);
  Serial.print("Pos2: ");
  Serial.println(m_pos2);
  Serial.print("Pos3: ");
  Serial.println(m_pos3);
  Serial.print("Pos4: ");
  Serial.println(m_pos4);
  Serial.print("Pincher pos: ");
  Serial.println(m_pincher_pos);

  
  #ifdef Arbotix
    // TODO -> linear movement with threads
    //unsigned int m_position[] = {4, m_pos1, m_pos2, m_pos3, m_pos4};
    //SetPosition(5, m_pincher_pos);
    //set_position_slowly(m_position);

    SetPosition(1, m_pos1);
    SetPosition(2, m_pos2);
    SetPosition(3, m_pos3);
    SetPosition(4, m_pos4);
	SetPosition(5, m_pincher_pos);
  #endif    

  
  // New run?
  Serial.println("Send a random character if you want to set another position");
  // Wait til the serial input
  while(!Serial.available()){}
  incomingByte = Serial.read();
  Serial.flush();
  delay(1000);

}


#ifdef Arbotix
//TODO 5 if pincher not needed
  void set_position_slowly(const unsigned int* p_position){
    delay(100);                    // recommended pause
    bioloid.loadPose(p_position);   // load the pose from FLASH, into the nextPose buffer
    bioloid.readPose();            // read in current servo positions to the curPose buffer   
    delay(100);
    bioloid.interpolateSetup(1000); // setup for interpolation from current->next over 1/2 a second
    while(bioloid.interpolating > 0){  // do this while we have not reached our new pose
      bioloid.interpolateStep();     // move servos, if necessary. 
      delay(3);
    }
  }
#endif


int calc_position_from_teta(double p_teta){
  if(isnan(p_teta))
    p_teta = 0.0;
  
  double m_degrees = 0.0;
  m_degrees = p_teta * (180.0/M_PI);

  m_degrees = m_degrees + 180.0;
	
  int m_pos = 0; // Possible positions: 0 - 1023
  double m_one_degree;
  
  // If it is in degree
  // Else -> degree shall be calculated first from the radian value
  m_one_degree = 1024.0 / 360.0; // Around 2.844...


  m_pos = (int)round(m_degrees*m_one_degree);  

  return m_pos;
}

double calc_teta1(double p_dx, double p_dy){
  //Teta1 calculation
  double m_teta1 = atan( p_dy / p_dx );
  
  return m_teta1;
}

double calc_teta2(double p_dz, double p_phi, double p_teta3){
  // Basic calculation for Teta2
  double m_a = L3 * sin(p_teta3);
  double m_b = L2 + ( L3 * cos(p_teta3) );
  double m_c = p_dz - L1 - ( L4 * sin(p_phi) );

  // R calculation based on m_a and m_b for Teta2
  double m_r = sqrt( (m_a*m_a)+(m_b*m_b) );
  
  // Teta2 calculation based on the other values
  // TODO2 define which one is the left handed
  #ifdef LEFT_HANDED  // If left handed arm
    double m_teta2 = atan2(m_c, sqrt((m_r*m_r) - (m_c*m_c)) ) - atan2(m_a, m_b);
  #else               // If right handed arm
    double m_teta2 = atan2(m_c, (-sqrt((m_r*m_r) - (m_c*m_c)) )) - atan2(m_a, m_b);    
  #endif

  return m_teta2;
}

double calc_teta3(double p_dx, double p_dy, double p_dz, double p_teta1, double p_phi){
  // R calculation based on m_a and m_b for Teta3
  double m_A = p_dx - (L4*cos(p_teta1)*cos(p_phi));
  double m_B = p_dy - (L4*sin(p_teta1)*cos(p_phi));
  double m_C = p_dz - L1 - (L4*sin(p_phi));

  // Teta3 calculation based on the other values
  double m_teta3 = acos( ( (m_A*m_A) + (m_B*m_B) + (m_C*m_C) - (L2*L2) - (L3*L3) ) / (2*L2*L3) );
  
  return m_teta3;
}

double calc_teta4(double p_phi, double p_teta2, double p_teta3){
  // Teta4 calculation based on the parameters
  double m_teta4 = p_phi - p_teta2 - p_teta3;
  
  return m_teta4;
}
