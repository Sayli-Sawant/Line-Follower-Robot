#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "motor_pwm.h"
#include "uart.h"
#define ADC_channels 3//1)reference 2)current 3)speed
int ADC_buff;
float duty1=0;
float duty2=0;
float dutytest1=0.8;
float dutytest2=0.8;
int duty_cycle1=80;
int duty_cycle2=80;
float ref;
int adc_value = 0;

void delayMs(int n);
void clock_configuration();
void ADC_SS3_ISR(void);
void timer0A_init(void);
void ADC_config();
void pwm_init_motor_right();
void motors_move_forward();

char str[80];
char mesg[12];
char vol_str[10];
char duty_str[10];
char ref_str[10];
int start_motor_flag=0;
int calculate_flag=0;
double prev_ref;
double set_point = 2.25; //voltage ip at mid pt
double total_error = 0;
double present_error;
double correction;
#define DUTY_MIN 0
#define DUTY_MAX 1
#define PWM_LOAD 4000
#define TIMER_LOAD 64000 //2ms
#define DUTY_SCALER 0.4 // 1.3, 54.23 sec, 1.55, 37.66 sec, 1.7, 32.23 sec
//#define LEFT_WHEEL_OFFSET 0.00125
#define ADC_PRIORITY 3    // Priority for ADC interrupts
#define TIMER_PRIORITY 2  // Priority for Timer interrupts

int main(){

    clock_configuration(); //working
    uart_init();
    printString("main\n");
    ADC_config(); //working
    timer0A_init(); //working


    pwm_init_motor_right();
    set_duty_cycle(50,50);
    config_port_a();
    motors_move_forward();
    //ADC_config();


    //NVIC_EN0_R = 0x00010000; //enable interrupt for ADC0 SS2
    //EnableInterrupts();
    //GPIO_PORTD_DIR_R |= 0x01;
    //GPIO_PORTD_DEN_R |= 0x01;
    //printString("\r Before NVIC \n");
     // Enable ADC3 handler
    //printString("\r After NVIC \n");
    printString("\r Going to While \n");
    while(1){
        //motors_move_forward();
        //printString("\rIn While\n");
//        ADC0_PSSI_R |= (1<<3);
//        while((ADC0_RIS_R & 8) ==0);
//        adc_value = ADC0_SSFIFO3_R;
//        ADC0_ISC_R=8;







        if(calculate_flag==1)
        {
            calculate_flag=0;
            duty1=(float)(0.5-correction);
            duty2 = (float)(0.62+correction);
            //set_duty_cycle((int)(duty1*100),(int)(duty2*100));
            //Left Wheel offset can be removed after placing the extra wheel in the middle to balance weights => currently it's biased
            //Clamp duty cycles to min and max valid values
            if(duty1<0)
            {
                duty1 = DUTY_MIN;
            }
            if(duty1>1)
            {
                duty1 = DUTY_MAX;
            }
            if(duty2<0)
            {
                duty2 = DUTY_MIN;
            }
            if(duty2>1)
            {
                duty2 = DUTY_MAX;
            }
            duty_cycle1=(int)(duty1*100);
            duty_cycle2=(int)(duty2*100);
            set_duty_cycle(duty_cycle1, duty_cycle2);
        }
        //delayMs(2000);
//


    }
}

void clock_configuration()
{
    /* Enable Peripheral Clocks */
    SYSCTL_RCGCPWM_R |= 2;       /* enable clock to PWM1 */
    SYSCTL_RCGCPWM_R |= 1;       /* enable clock to PWM0 */
    for(int i=0;i<100;i++);
    SYSCTL_RCGCGPIO_R |= 0x38;   /* enable clock to PORTF and PORTD and PORTE*/
    for(int i=0;i<100;i++);
    SYSCTL_RCC_R &= ~0x001C0000; /* pre-divide for PWM0 clock - by 64 */
    SYSCTL_RCC_R |= 0x00100000;
    for(int i=0;i<100;i++);
    SYSCTL_RCGCADC_R |= 1;       /* enable clock to ADC0 */
}


void ADC_config()
{
    //here we will be using PE3(AIN0),PE2(AIN1),PE1(AIN2) pins for the analog inputs
    /* initialize ADC0 */
    GPIO_PORTE_AFSEL_R |= (1<<3);   /* enable alternate function */
    GPIO_PORTE_DEN_R &= ~(1<<3);    /* disable digital function */
    GPIO_PORTE_AMSEL_R |= (1<<3);   /* enable analog function */
    ADC0_ACTSS_R &= ~(1<<3);          /* disable SS3 during configuration */
    ADC0_EMUX_R &= ~0xF000;
    //ADC0_EMUX_R |= 0x0800;       /* PWM0 gen2 trigger conversion seq  */
    //ADC0_TSSEL_R&=~0x00300000;
    ADC0_SSMUX3_R = 0;           /* get input from channel 0 */
    //ADC0_PC_R = 0x03;
    ADC0_SSCTL3_R |= ((1<<1)|(1<<2));       /* take one value, set flag at 1st sample */
    ADC0_IM_R|=(1<<3);//Sends the raw interrupt to the interrupt manager upon conversion
    ADC0_ACTSS_R |= (1<<3);           /* enable ADC0 sequencer 3 */
    NVIC_PRI4_R = (NVIC_PRI4_R & 0xFFFFFF00) | (ADC_PRIORITY << 5);
    ADC0_PSSI_R |= (1<<3);
    NVIC_EN0_R |= 0x00020000;
}

void timer0A_init(void)
{
    SYSCTL_RCGCTIMER_R |= 1;
    TIMER0_CTL_R = 0;  // DISABLE TIMER
    TIMER0_CFG_R = 0x04;  //16 BIT MODE
    TIMER0_TAMR_R = 0x02;  // DOWN COUNT AND PERIODIC
    TIMER0_TAPR_R = 16 - 1;  //PRESCALER VALUE
    TIMER0_TAILR_R = TIMER_LOAD - 1;  //LOAD VALUE - MACRO
    TIMER0_ICR_R = 0x1;  // CLEARING TIMEOUT INTERRUPTS
    TIMER0_IMR_R = 0X00000001;  //TIMEOUT ENABLE
    NVIC_PRI4_R = (NVIC_PRI4_R & 0xFFFF00FF) | (TIMER_PRIORITY << 13);
    NVIC_EN0_R |= 0x00080000; // ENABLING PEERIODIC INTERRUPTS FROM TIMER0
    TIMER0_CTL_R |= 0x01; //ENABLE TIMER AND START COUNTING
    //EnableInterrupts();
}

//void EnableInterrupts(void)
//{
//    __asm  ("    CPSIE  I\n");
//}

//void PWM0GEN2_ISR()
//{
//    PWM0_2_ISC_R|=0x00000008;
//}

void ADC_SS3_ISR(void)
{
    //printString("\rIn ADC handler\n");
    int i=0;
    /*while(i<ADC_channels && i < sizeof(ADC_buff) / sizeof(ADC_buff[0]))
    {
        ADC_buff[i] = ADC0_SSFIFO3_R;  //(i=0||PE3)ref (i=1||PE2)omega (i=2||PE1)current
        i++;
    }*/
    ADC_buff = ADC0_SSFIFO3_R;
    ref=(float)(ADC_buff*3.3)/(float)4096;+++++++++++++++++++++++++++++++++++
    gcvt(ref,5,ref_str);
    printString("\r Reference: ");
    printString(ref_str);
    printString("\n");
    ADC0_ISC_R |= 0x08;
    calculate_flag=1;
    //ADC0_PSSI_R |= (1<<3);
}

void timer0_handler(void)
{
    //printString("timer handler\n");
    //pid control for duty cycles
    //printString("\rIn timer handler\n");
    //double Kp = 0.01, Ki = 0.005, Kd = 0.002;//0.48 //0.0
    double Kp = 0.002, Ki = 0.005, Kd = 0.002;
    double present_error,dt = 0.002;   //dt=2 ms which timer overflow interrupt
    TIMER0_ICR_R = 0x1;  //clear timeout

    double Deviation = -(prev_ref - set_point) - (ref - set_point);
    prev_ref = ref;
    present_error = (set_point - ref);

    //PID algorithm
    correction = Kp * (present_error); //+ Ki * total_error + Kd * Deviation/dt;
    total_error = total_error + present_error*dt;
    ADC0_PSSI_R |= (1<<3);

}

void delayMs(int n)
{
    int i, j;

    for(i = 0 ; i < n; i++)
        for(j = 0; j < 3180; j++)
            {}  /* do nothing for 1 ms */
}
