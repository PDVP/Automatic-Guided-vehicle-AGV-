

float motor_dis=0.10;
float wheel_rad=0.035; //in meters


float vt_1 = 0.00;
float vt_2 = 0.00;

float v_teleop=0.00;
float w_teleop=0.00;
ros::NodeHandle nh;

void messageCb( const geometry_msgs::Twist& msg){
  
  v_teleop = msg.linear.x;
  w_teleop = msg.angular.z;

}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
nh.initNode();
 nh.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:
nh.spinOnce();
Serial.print("lin = ");
  Serial.print(v_teleop);
  Serial.print("\t ");
  Serial.print("ang = ");
  Serial.print(w_teleop);
  Serial.println();

  vt_1 = (( (2.00*v_teleop) + (w_teleop)*motor_dis)/(2.00*wheel_rad) ); //if very less the multipy to get a finite velocity
  vt_2 = (( (2.00*v_teleop) - (w_teleop)*motor_dis)/(2.00*wheel_rad) );
  Serial.print("v1 = ");
  Serial.print(vt_1);
  Serial.print("\t ");
  Serial.print("v2 = ");
  Serial.print(vt_2);
  Serial.println();
}
