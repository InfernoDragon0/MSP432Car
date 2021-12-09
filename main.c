#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ESP8266.h>
#include <UART_Driver.h>
#define MIN_DISTANCE    50.0f
#define TICKPERIOD      1000

uint32_t SR04IntTimes;
static volatile bool flipFlop;
uint32_t notchesdetectedLeft;
uint32_t notchesdetectedRight;

uint32_t currentSpeed = 0;

eUSCI_UART_ConfigV1 UART0Config =
{
     EUSCI_A_UART_CLOCKSOURCE_SMCLK,
     13,
     0,
     37,
     EUSCI_A_UART_NO_PARITY,
     EUSCI_A_UART_LSB_FIRST,
     EUSCI_A_UART_ONE_STOP_BIT,
     EUSCI_A_UART_MODE,
     EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
};

eUSCI_UART_ConfigV1 UART2Config =
{
     EUSCI_A_UART_CLOCKSOURCE_SMCLK,
     13,
     0,
     37,
     EUSCI_A_UART_NO_PARITY,
     EUSCI_A_UART_LSB_FIRST,
     EUSCI_A_UART_ONE_STOP_BIT,
     EUSCI_A_UART_MODE,
     EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
};

Timer_A_PWMConfig pwmConfigLeft =
{
     TIMER_A_CLOCKSOURCE_SMCLK,
     TIMER_A_CLOCKSOURCE_DIVIDER_48,
     20000,
     TIMER_A_CAPTURECOMPARE_REGISTER_1,
     TIMER_A_OUTPUTMODE_RESET_SET,
     0
};

Timer_A_PWMConfig pwmConfigRight =
{
     TIMER_A_CLOCKSOURCE_SMCLK,
     TIMER_A_CLOCKSOURCE_DIVIDER_48,
     20000,
     TIMER_A_CAPTURECOMPARE_REGISTER_2,
     TIMER_A_OUTPUTMODE_RESET_SET,
     0
};
/**
 * main.c
 */

//AT Command helpers
bool ESP8266_SETAP3(void);
bool ESP8266_GETIP(void);
void Initalise_HCSR04(void);
float getHCSR04Distance(void);
static void Delay(uint32_t loop);
void uPrintf(unsigned char * TxArray);

void addSpeed(void);
void reduceSpeed(void);
void moveStop(void);
void moveForward(void);
void moveBack(void);
void moveLeft(void);
void moveRight(void);
void sendData(void);

void main(void)
{
    MAP_WDT_A_holdTimer();
    notchesdetectedLeft = 0;
    notchesdetectedRight = 0;

    /*Ensure MSP432 is Running at 24 MHz*/
    FlashCtl_setWaitState(FLASH_BANK0, 2);
    FlashCtl_setWaitState(FLASH_BANK1, 2);
    PCM_setCoreVoltageLevel(PCM_VCORE1);
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_24);

    /*Initialize required hardware peripherals for the ESP8266*/
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_UART_initModule(EUSCI_A0_BASE, &UART0Config);
    MAP_UART_enableModule(EUSCI_A0_BASE);
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_UART_initModule(EUSCI_A2_BASE, &UART2Config);
    MAP_UART_enableModule(EUSCI_A2_BASE);
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA2);

    //wheel encoder interrupts
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3, GPIO_PIN6);
    GPIO_clearInterruptFlag(GPIO_PORT_P3, GPIO_PIN6);
    GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN6);

    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3, GPIO_PIN7);
    GPIO_clearInterruptFlag(GPIO_PORT_P3, GPIO_PIN7);
    GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN7);


    //test interrupt
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

    Interrupt_enableInterrupt(INT_PORT3);
    Interrupt_enableInterrupt(INT_PORT1);

    /* GPIO as power source */
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN2);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0);

    //Motor
    //P2.4 > In1
    //P2.5 > In2
    //P2.6 > In3
    //P2.7 > In4
    //P5.6 > EMA
    //P5.7 > EMB
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN4);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN5);

    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN6);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7);

    //moving forwards, 4,5 is right, 6,7 is left
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN4);

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);


    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigLeft);
    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigRight);

    /*Reset GPIO of the ESP8266*/
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN1);

    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2); //Led2s
    MAP_Interrupt_enableMaster();

    Initalise_HCSR04();
    //turn off all the led lights
    P2OUT &= ~0x07;
    UART_Printf(EUSCI_A0_BASE, "helotest\r\n");
    //setup AP 3rd mode
    //AT_CWMODE=3
    bool cwCheck = ESP8266_SETAP3();
    if (cwCheck) {
        //led White blink 3 times
        //P2OUT |= BIT0; //red on forever
        UART_Printf(EUSCI_A0_BASE, "cw set success\r\n");
    }
    else {
        //led white stays on
        UART_Printf(EUSCI_A0_BASE, "cw set error\r\n");
    }

    UART_Printf(EUSCI_A0_BASE, "connecting to ap\r\n");

    if(ESP8266_AvailableAPs()) {
        UART_Printf(EUSCI_A0_BASE, ESP8266_GetBuffer());
    }
    else {
        UART_Printf(EUSCI_A0_BASE, "no ap found\r\n");
    }
    //then connect to some wifi (maybe have an input thru esp terminal?)
    //AT+CWJAP="SSID","password"
    bool apCheck = ESP8266_ConnectToAP("ssid", "password");
    if (apCheck) {
        //if ok, blink the green LED 3 times, then turn off green led
        //P2OUT |= BIT1;
        UART_Printf(EUSCI_A0_BASE, "ap connect success\r\n");
    }
    else {
        //if not, green led stays on
        UART_Printf(EUSCI_A0_BASE, ESP8266_GetBuffer());
        //while(1);
    }

    __delay_cycles(48000000);
    ESP8266_GETIP();

    UART_Printf(EUSCI_A0_BASE, "connecting to tcp\r\n");
    //then connect to the nextjs server websocket via TCP
    //AT+CIPSTART="TCP","rcserver.infernodragon.cloud",9100

    //ESP8266_Terminal();

    bool connectCheck = ESP8266_EstablishConnection('0', TCP, "rcserver.infernodragon.cloud", "20001");
    if (connectCheck) {
        //if ok, blink cyan 3 times
        UART_Printf(EUSCI_A0_BASE, "TCP Ok\r\n");
        //P2OUT |= BIT2;
    }
    else {
        //if not, cyan led stays on
        UART_Printf(EUSCI_A0_BASE, "tcp failed\r\n");
        __delay_cycles(48000000);
        UART_Printf(EUSCI_A0_BASE, ESP8266_GetBuffer());
        //while(1);
    }


    //send data to server
    //AT+CIPSEND=5 (length on bytes)
    //hello (actual data)
    char hs[100];
    sprintf(hs, "{\"op\":1,\"s\":%d,\"n\":\"PCar\",\"o\":0}", currentSpeed);
    bool handshake = ESP8266_SendData('0', hs, 31); //currentspeed is always 0 at start

    if (handshake) {
        //if ok, blink red 3 times
        //recv handshake

    }
    else {
        //if not, red stays on
    }
    //recv data from server
    //+IPD,n:xxxxxxxxxx (n bytes, xx data)
    char *ESP8266_Data = ESP8266_GetBuffer();
    UART_Printf(EUSCI_A0_BASE, "data from handshake\r\n");
    UART_Printf(EUSCI_A0_BASE, ESP8266_GetBuffer());
    //MSPrintf(EUSCI_A0_BASE, "ESP8266 Data Received: %s\r\n", ESP8266_Data);
    //end later
    //AT+CIPCLOSE
    while(1) {
        //listen to the server
        __delay_cycles(6000000);

        //ultrasonic sensor

        if((getHCSR04Distance() < MIN_DISTANCE)) {
            UART_Printf(EUSCI_A0_BASE, "HCSR04 reaching min distance, stopping\r\n");

            pwmConfigLeft.dutyCycle = 0;
            pwmConfigRight.dutyCycle = 0;
            Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigLeft);
            Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigRight);
        }

        else {
            char buf[100];
            sprintf(buf, "data=%.2f\r\n", getHCSR04Distance());
            //UART_Printf(EUSCI_A0_BASE, buf);

        }

        if(!ESP8266_WaitForAnswer(ESP8266_RECEIVE_TRIES))
        {
            //do nothing
            UART_Printf(EUSCI_A0_BASE, "tcpNA\r\n");
            continue;
        }

        //switch here
        //movements
        UART_Printf(EUSCI_A0_BASE, ESP8266_GetBuffer());
        if (strstr(ESP8266_GetBuffer(), "W") != NULL) {
            UART_Printf(EUSCI_A0_BASE, "move forward\r\n");
            moveForward();
        }
        else if (strstr(ESP8266_GetBuffer(), "S") != NULL) {
            UART_Printf(EUSCI_A0_BASE, "move back\r\n");
            moveBack();
        }
        else if (strstr(ESP8266_GetBuffer(), "A") != NULL) {
            UART_Printf(EUSCI_A0_BASE, "move left\r\n");
            moveLeft();
        }
        else if (strstr(ESP8266_GetBuffer(), "DD") != NULL) {
            UART_Printf(EUSCI_A0_BASE, "move right\r\n");
            moveRight();
        }

        else if (strstr(ESP8266_GetBuffer(), "X") != NULL) {
            UART_Printf(EUSCI_A0_BASE, "stop moving\r\n");
            moveStop();
        }

        else if (strstr(ESP8266_GetBuffer(), "Q") != NULL) {
            UART_Printf(EUSCI_A0_BASE, "add speed\r\n");
            addSpeed();
        }

        else if (strstr(ESP8266_GetBuffer(), "E") != NULL) {
            UART_Printf(EUSCI_A0_BASE, "reduce speed\r\n");
            reduceSpeed();
        }

        //data and reset
        else if (strstr(ESP8266_GetBuffer(), "Z") != NULL) {
            UART_Printf(EUSCI_A0_BASE, "send all data\r\n");
            sendData();
        }
        else if (strstr(ESP8266_GetBuffer(), "C") != NULL) {
            UART_Printf(EUSCI_A0_BASE, "reset speed and variables\r\n");
        }

        else {
            UART_Printf(EUSCI_A0_BASE, "got default\r\n");
            UART_Printf(EUSCI_A0_BASE, ESP8266_GetBuffer());

        }

    }
}

void TA0_0_IRQHandler(void)
{
    /* Increment global variable (count number of interrupt occurred) */
    SR04IntTimes++;

    /* Clear interrupt flag */
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

// -------------------------------------------------------------------------------------------------------------------

static uint32_t getHCSR04Time(void)
{
    uint32_t pulsetime=0;

    /* Number of times the interrupt occurred (1 interrupt = 1000 ticks)    */
    pulsetime = SR04IntTimes * TICKPERIOD;

    /* Number of ticks (between 1 to 999) before the interrupt could occur */
    pulsetime += Timer_A_getCounterValue(TIMER_A0_BASE);

    /* Clear Timer */
    Timer_A_clearTimer(TIMER_A0_BASE);

    Delay(3000);

    return pulsetime;
}

void PORT3_IRQHandler(void)
{
    uint32_t status;
    //UART_Printf(EUSCI_A0_BASE, "wheel interrupt\r\n");
    status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P3);

    if (status & GPIO_PIN6) {
        notchesdetectedLeft++;

        if(notchesdetectedLeft == 20)
        {
            UART_Printf(EUSCI_A0_BASE, "wheel Left spinned fully\r\n");
            notchesdetectedLeft = 0;
        }
    }

    if (status & GPIO_PIN7) {
        notchesdetectedRight++;

        if(notchesdetectedRight == 20)
        {
            UART_Printf(EUSCI_A0_BASE, "wheel Right spinned fully\r\n");
            notchesdetectedRight = 0;
        }
    }

    GPIO_clearInterruptFlag(GPIO_PORT_P3, status);
}

void PORT1_IRQHandler(void)
{
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    UART_Printf(EUSCI_A0_BASE, "switch1 pressed\r\n");
    pwmConfigLeft.dutyCycle = 0;
    pwmConfigRight.dutyCycle = 0;
    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigLeft);
    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigRight);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

}

bool ESP8266_SETAP3(void)
{
    UART_Printf(EUSCI_A2_BASE, "AT+CWMODE=3\r\n");
    __delay_cycles(48000000);
    if(!ESP8266_WaitForAnswer(ESP8266_RECEIVE_TRIES))
    {
        return false;
    }

    if(strstr(ESP8266_GetBuffer(), "OK") == NULL)
    {
        return false;
    }

    return true;
}

bool ESP8266_GETIP(void)
{
    UART_Printf(EUSCI_A2_BASE, "AT+CIFSR\r\n");
    __delay_cycles(48000000);
    if(!ESP8266_WaitForAnswer(ESP8266_RECEIVE_TRIES))
    {
        return false;
    }

    UART_Printf(EUSCI_A0_BASE, ESP8266_GetBuffer());

    return true;
}

void Initalise_HCSR04(void)
{
    /* Timer_A UpMode Configuration Parameter */
    const Timer_A_UpModeConfig upConfig =
    {
            TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
            TIMER_A_CLOCKSOURCE_DIVIDER_3,          // SMCLK/3 = 1MHz
            TICKPERIOD,                             // 1000 tick period
            TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
            TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
            TIMER_A_DO_CLEAR                        // Clear value
    };

    int a = CS_getSMCLK();

    /* Configuring P4.1 as trigger */
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN1);//
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);//

    /* P4.3 echo */
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P4, GPIO_PIN3);


    /* Configuring Timer_A0 for Up Mode */
    Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);

    /* Enabling interrupts and starting the timer */
    Interrupt_enableInterrupt(INT_TA0_0);
    //Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

    //Timer_A_stopTimer(TIMER_A0_BASE);
    Timer_A_clearTimer(TIMER_A0_BASE);

}

float getHCSR04Distance(void)
{
    uint32_t pulseduration = 0;
    float calculateddistance = 0;

    /* Generate 10us pulse at trig */
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1);
    Delay(30);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);

    /* Wait for positive-edge */
    while(GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN3) == 0);

    /* Start Timer */
    SR04IntTimes = 0;
    Timer_A_clearTimer(TIMER_A0_BASE);
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

    /* Detects negative-edge */
    while(GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN3) == 1);

    /* Stop Timer */
    Timer_A_stopTimer(TIMER_A0_BASE);

    /* Obtain Pulse Width in microseconds */
    pulseduration = getHCSR04Time();

    /* Calculating distance in cm */
    calculateddistance = (float)pulseduration / 58.0f;

    return calculateddistance;
}

static void Delay(uint32_t loop)
{
    volatile uint32_t i;

    for (i = 0 ; i < loop ; i++);
}

//car mobility
void addSpeed(void) {
    currentSpeed += 1000;

    if (currentSpeed <= 4000) {
        currentSpeed = 4000;
    }

    if (currentSpeed > 7000) {
        currentSpeed = 7000;
    }

    pwmConfigLeft.dutyCycle = currentSpeed;
    pwmConfigRight.dutyCycle = currentSpeed;
    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigLeft);
    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigRight);
}

void reduceSpeed(void) {

    if (currentSpeed <= 4000) {
        currentSpeed = 0;
    }
    else {
        currentSpeed -= 1000;
    }

    pwmConfigLeft.dutyCycle = currentSpeed;
    pwmConfigRight.dutyCycle = currentSpeed;
    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigLeft);
    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigRight);

}

void moveStop(void) { //stop duty cycle
    pwmConfigLeft.dutyCycle = 0;
    pwmConfigRight.dutyCycle = 0;
    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigLeft);
    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigRight);
}

void moveForward(void) {
    //moving forwards, 4,5 is right, 6,7 is left
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN4);

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);

    pwmConfigLeft.dutyCycle = currentSpeed;
    pwmConfigRight.dutyCycle = currentSpeed;
    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigLeft);
    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigRight);

    //send OK
    //char hs[100];
    //sprintf(hs, "{\"op\":1,\"s\":%d,\"n\":\"PCar\",\"o\":0}", currentSpeed);
    //bool sendResult = ESP8266_SendData('0', "{\"op\":3, \"cmd\":\"W\"}", 19);
}

void moveBack(void) {
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN6);

    pwmConfigLeft.dutyCycle = currentSpeed;
    pwmConfigRight.dutyCycle = currentSpeed;
    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigLeft);
    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigRight);

    //bool sendResult = ESP8266_SendData('0', "{\"op\":3, \"cmd\":\"S\"}", 19);
}

void moveLeft(void) {

    pwmConfigLeft.dutyCycle = currentSpeed;
    pwmConfigRight.dutyCycle = 0;
    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigLeft);
    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigRight);

    //bool sendResult = ESP8266_SendData('0', "{\"op\":3, \"cmd\":\"A\"}", 19);
}

void moveRight(void) {

    pwmConfigLeft.dutyCycle = 0;
    pwmConfigRight.dutyCycle = currentSpeed;
    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigLeft);
    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigRight);

    //bool sendResult = ESP8266_SendData('0', "{\"op\":3, \"cmd\":\"D\"}", 19);
}

void sendData(void) {
    char dat[100];
    sprintf(dat, "{\"op\":2,\"s\":%d,\"n\":\"PCar\",\"o\":0}", currentSpeed);
    bool sendResult = ESP8266_SendData('0', dat, currentSpeed == 0 ? 31 : 34);
}
