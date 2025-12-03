#include<Arduino.h>
#include"system_performance.h"
void SysPer_init(sys_per* system, sys_criteria* criteria, double setpoint, double Ess, double POT, double Tr, double Tss){
  criteria->final_error = Ess;
  criteria->overshoot = POT;
  criteria->time_rise = Tr;
  criteria->time_settle = Tss;
  criteria->setpoint = setpoint;

  system->final_val =0;
  system->final_error =0;
  system->overshoot =0;
  system->time_rise =0;
  system->time_settle =0;
  system->counter =0;
  system->highest_val =0;
  system->flag_settle =0;
  system->flag_risen =0;
  system->flag_risen_low =0;
  system->timer =0;
  for (int i =0; i <10; i++){
    system->cached[i] = 0;
  }
}

bool evaluate(sys_per* system, sys_criteria criteria, double val, unsigned long runtime) {
  //steady state = average of 3 last value
  system->cached[system->counter] = val;
  system->counter = (system->counter + 1) % 10;
  double sum =0;
  for(unsigned int i =0; i < 10; i++){
    sum += system->cached[i];
  }
  system->final_val = sum/10;
  system->final_error = criteria.setpoint - system->final_val;
  if(system->final_error < 0){
    system->final_error = -system->final_error;
  }
  //overshoot calculate
  if(system->highest_val < val){
    system->highest_val = val;
  }
  else {
    if(system->final_val != 0){
      system->overshoot = (system->highest_val - system->final_val)*100/system->final_val;
    }
  }
  //time rise calculate
  if (val >= criteria.setpoint*0.1 && val <=criteria.setpoint*0.9){
    if(!system->flag_risen){
      system->timer = runtime;
    }
    else system->time_rise = runtime - system->time_rise;
  }
  //time settle calculate
  double rate = 0.05;
  if((val >= criteria.setpoint*(1- rate) && val <= criteria.setpoint*(1+rate)) && system->flag_settle != 1){
    system->time_settle = runtime*0.001;
    system->flag_settle = 1;
  }
  else if(val < criteria.setpoint*(1- rate) || val > criteria.setpoint*(1 + rate)){
    system->flag_settle = 0;
  }
  //return value
  if ((system->final_error <= criteria.final_error) && system->overshoot <= criteria.overshoot && system->time_rise <= criteria.time_rise && system->time_settle <= criteria.time_settle){
    return 1;
  }
  else{
    return 0;
  }
}
//for import to csv
void printCSV(float x, float y) {
  Serial.print((float)x, 2);
  Serial.print(",");
  Serial.println((float)y, 2);
}
//for teleplot extension
void teleplot(double time, double signal){
  Serial.print(">setpoint:"); Serial.println(time);
  Serial.print(">signal:"); Serial.println(signal); 
}
//print system evaluation in Json
void print_performance(sys_per sys, sys_criteria criteria, double limit){
  //Json format
  Serial.println(" {\"evaluation\":{");
  Serial.print("\"steady state error\":");Serial.print("\"");Serial.print(sys.final_error);Serial.println("\",");
  Serial.print("\"steady state value\":");Serial.print("\"");Serial.print(sys.final_val);Serial.println("\",");
  Serial.print("\"overshoot\":");Serial.print("\"");Serial.print(sys.overshoot);Serial.println("\",");
  Serial.print("\"time rise\":");Serial.print("\"");
  if (sys.time_rise > 1e-4) {Serial.print(sys.time_rise);}
  else Serial.print("invalid"); 
  Serial.println("\",");
 
  Serial.print("\"time settle\":");Serial.print("\"");  
  if (sys.time_settle <= limit) {Serial.print(sys.time_settle);}
  else Serial.print("inf"); 
  Serial.println("\"");

  Serial.print(" "); Serial.println(" }");
  Serial.println("}");
  //Serial.println(" {\"evaluation\":{");
/*  
  //CSV format
//  Serial.print("\"steady state error\":");Serial.print("\"");
  Serial.print(sys.final_error);Serial.print(", ");
//  Serial.print("\"steady state value\":");Serial.print("\"");
  Serial.print(sys.final_val);Serial.print(", ");
//  Serial.print("\"overshoot\":");Serial.print("\"");
  Serial.print(sys.overshoot);Serial.print(", ");
//  Serial.print("\"time rise\":");Serial.print("\"");
  Serial.print(sys.time_rise);

  Serial.print(" ,");
 */
//  Serial.print("\"time settle\":");Serial.print("\"");  
  if (sys.time_settle <= limit) {Serial.print(sys.time_settle);}
  else Serial.print("inf"); 
  Serial.println("");


  }
void print_criteria(sys_criteria criteria){
  Serial.println(" {\"criteria\":{");
  Serial.print("\"steady state error\":");Serial.print("\"");Serial.print(criteria.final_error);Serial.println("\",");
  Serial.print("\"steady state value\":");Serial.print("\"");Serial.print(criteria.setpoint);Serial.println("\",");
  Serial.print("\"overshoot\":");Serial.print("\"");Serial.print(criteria.overshoot);Serial.println("\",");
  Serial.print("\"time rise\":");Serial.print("\""); Serial.print(criteria.time_rise); Serial.println("\"");
 
  Serial.print("\"time settle\":");Serial.print("\"");  
Serial.print(criteria.time_settle); Serial.println("\"");

  Serial.print(" "); Serial.println(" }");
  Serial.println("}");
}
bool meetCriteria(sys_per sys, sys_criteria criteria){
  bool satisfy = (sys.overshoot <= criteria.overshoot) && (sys.final_error <= criteria.final_error) && (sys.time_rise <= criteria.time_rise) && (sys.time_settle <= criteria.time_settle);
  return satisfy;
}
bool reject(sys_per sys, sys_criteria criteria){
  bool rej = sys.time_rise > criteria.time_rise || sys.time_settle > criteria.time_settle;
  return rej;
}