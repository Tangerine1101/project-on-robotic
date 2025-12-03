//this library is for evaluation and handle data
#ifndef SYSTEM_PERFORMANCE_H
#define SYSTEM_PERFORMANCE_H

struct sys_criteria {
  double overshoot;    //%
  double final_error; //steady state error
  double time_rise;   //time needed to rise from 10% to 90% 
  double time_settle; //time needed to read final value with 5% error
  double setpoint;
};

struct sys_per {
  double overshoot;    //
  double final_val;   //steady state value
  double final_error; //steady state error
  double highest_val; //highest value
  double time_rise;   //time needed to rise from 10% to 90% 
  double time_settle; //time needed to read final value with 5% error
  double cached[10];   //cached to calculate steady state value
  unsigned int counter; 
  unsigned long timer;
  bool flag_settle;
  bool flag_risen, flag_risen_low;
};

void SysPer_init(sys_per* system, sys_criteria* criteria, double setpoint, double Ess, double POT, double Tr, double Tss);
bool evaluate(sys_per* system, sys_criteria criteria, double val, unsigned long runtime);
void printCSV(float x, float y);
void print_performance(sys_per sys, sys_criteria criteria, double limit);
void teleplot(double time, double value);
bool meetCriteria(sys_per sys, sys_criteria criteria);
void print_criteria(sys_criteria criteria);
bool reject(sys_per sys, sys_criteria criteria);
#endif