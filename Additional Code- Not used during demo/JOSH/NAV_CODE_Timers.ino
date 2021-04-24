

// the number of the PWM pin

const uint8_t PinL  = 12;  // 12 corresponds to GPIO12 

const uint8_t PinR  = 13;  // 13 corresponds to GPIO13 

const uint8_t PinL_in1  = 2;  // 12 corresponds to GPIO2 

const uint8_t PinL_in2  = 15;  // 13 corresponds to GPIO15 

const uint8_t PinR_in3  = 18;  // 12 corresponds to GPIO18 

const uint8_t PinR_in4  = 4;  // 13 corresponds to GPIO4 


// setting PWM properties

const uint32_t freq = 5000;

const uint8_t L_Channel = 1; // note channel 0 and 1 share frequencies

const uint8_t R_Channel = 2; // note channel 2 and 3 share frequencies

const uint8_t resolution = 8;


double  t0 = millis()/1000.0;

double  t = t0;

double  t_straight = 10.0;

double  t_turn_90_deg = 5.0;

double  t_both = t_straight + t_turn_90_deg;

double  tmax = 180.0;

double  t_reduce = 1.0;

int     straight_leg = 1;

int     first_time_through = 1;

int     leg_num = 1;


void setup() {

   

    // put your setup code here, to run once:


    pinMode(PinL_in1, OUTPUT);

    pinMode(PinL_in2, OUTPUT);

    pinMode(PinR_in3, OUTPUT);

    pinMode(PinR_in4, OUTPUT);


    ledcAttachPin(12,1); //( PinL, L_Channel);

    ledcAttachPin(13,2); //( PinR, R_Channel);


    ledcSetup(1,5000,8);//( L_Channel, freq, resolution);

    ledcSetup(2,5000,8);//( R_Channel, freq, resolution);


    Serial.begin(9600);

   

 

}


void loop() {


    // put your main code here, to run repeatedly:

   

    t = millis()/1000.0 - t0;


    if( first_time_through == 1){

         //Setup left motor driver logic for PWM

         digitalWrite(PinL_in1, HIGH);

         digitalWrite(PinL_in2, LOW);

 

        //Reverse for right motor

        digitalWrite(PinR_in3, LOW);

        digitalWrite(PinR_in4, HIGH);

        delay(50);


        //Sets PWM (channel 1 is left, channel 2 is right)

        ledcWrite(1,255);

        ledcWrite(2,255);


    first_time_through = 0;


    }


    if( straight_leg==1 &&

    ( floor(t)==floor(t_straight)  ||

     ((int)(t-t_straight) % (int)t_both) == 0  )){


    straight_leg = 0;


        // Reverse right motor for 5 sec

        //ledcWrite(2,128);

        digitalWrite(PinL_in1, LOW);
        
        digitalWrite(PinR_in4, LOW);

        delay(2000);

        digitalWrite(PinL_in1, HIGH);

        digitalWrite(PinR_in3, HIGH);

        
        

    }


    if( straight_leg==0 &&

        ( floor(t)==floor(t_both) ||

          ((int)(t-t_both) % (int)t_both) == 0 )){


 

    // Turn ON right motor for 10 sec

        if( t < tmax){ // after tmax seconds do not turn right motor back on ***

            digitalWrite( PinR_in3, LOW);

            digitalWrite( PinR_in4, HIGH);

            //ledcWrite(2,255);
            

            straight_leg = 1;

            leg_num = leg_num + 1;

           

            if(leg_num > 3){


                t_straight = t_straight - ((int)(leg_num+1) % (int)2)*t_reduce;

           

                t_both = t_straight + t_turn_90_deg;


            }

           

            

        }

        else{

           digitalWrite( PinL_in1, LOW);  // turn off left motor after tmax seconds ***

           digitalWrite( PinR_in3, LOW);  // turn off right motor after tmax seconds ***

        }

    }


}
