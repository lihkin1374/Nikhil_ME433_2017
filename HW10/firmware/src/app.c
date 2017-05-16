/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c
  Summary:
    This file contains the source code for the MPLAB Harmony application.
  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.
Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).
You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.
SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include <stdio.h>
#include <xc.h>
#include"i2c_master_noint.h"    //include i2c header file
#include"ILI9163C.h"    //include LCD header file

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

uint8_t APP_MAKE_BUFFER_DMA_READY dataOut[APP_READ_BUFFER_SIZE];
uint8_t APP_MAKE_BUFFER_DMA_READY readBuffer[APP_READ_BUFFER_SIZE];
int len, i = 0;
int startTime = 0;

char msg[100];
static int printData = 0;
unsigned char data[14];
signed short data_array[7];
int j = 0, k = 0, n = 0;
int elapsedTime = 0;
int coInd = 0;


int mafSum = 0;
int mafAve = 0;
int sampleIndex = 0;
float prevAve = 0;
float newAve = 0;
float iirAve = 0;

float firAve = 0;

// *****************************************************************************
/* Application Data
  Summary:
    Holds application data
  Description:
    This structure holds the application's data.
  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
 */

APP_DATA appData;

#define SLAVE_ADDR 0x6B //define slave address
#define CTRL1_ADDR 0x10 //define address of CTRL_1 register
#define CTRL2_ADDR 0x11 //define address of CTRL_1 register
#define CTRL3_ADDR 0x12 //define address of CTRL_1 register
#define OUT_TEMP_L_ADDR 0x20   //define address of OUT_TEMP_L register
#define OUTX_L_G_ADDR 0x22     //define address of OUTX_L_G register
#define OUTX_L_XL_ADDR 0x28    //define address of OUTX_L_XL register
#define OUTZ_L_XL_ADDR 0x2C    //define address of OUTX_L_XL register
#define WHO_AM_I_ADDR 0x0F  //define address of WHO_AM_I register

#define CS LATBbits.LATB7  //set chip select pin as B7
#define BAR_LENGTH 50  //set length of progress bar
#define CAL 500 //set scaling of value
#define BAR_WIDTH 2     //set width of progress bar
#define BACKGROUND ORANGE
#define TEXT BLACK

#define SAMPLE_NO 1000
#define MAX_LENGTH 5
#define ALPHA 0.95
#define BETA 0.05

int mafCalc[MAX_LENGTH] = {0};
float firCoeff[MAX_LENGTH] = {0.0201, 0.2309, 0.4981, 0.2309, 0.0201};
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */

/*******************************************************
 * USB CDC Device Events - Application Event Handler
 *******************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler
(
        USB_DEVICE_CDC_INDEX index,
        USB_DEVICE_CDC_EVENT event,
        void * pData,
        uintptr_t userData
        ) {
    APP_DATA * appDataObject;
    appDataObject = (APP_DATA *) userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;

    switch (event) {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(appDataObject->deviceHandle,
                    &appDataObject->getLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
                    &appDataObject->setLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *) pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration */

            appDataObject->breakData = ((USB_DEVICE_CDC_EVENT_DATA_SEND_BREAK *) pData)->breakDuration;

            /* Complete the control transfer by sending a ZLP  */
            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

            /* This means that the host has sent some data*/
            appDataObject->isReadComplete = true;
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. For now we accept all the data */

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the data write got completed. We can schedule
             * the next read. */

            appDataObject->isWriteComplete = true;
            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/***********************************************
 * Application USB Device Layer Event Handler.
 ***********************************************/
void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context) {
    USB_DEVICE_EVENT_DATA_CONFIGURED *configuredEventData;

    switch (event) {
        case USB_DEVICE_EVENT_SOF:

            /* This event is used for switch debounce. This flag is reset
             * by the switch process routine. */
            appData.sofEventHasOccurred = true;
            break;

        case USB_DEVICE_EVENT_RESET:

            /* Update LED to show reset state */

            appData.isConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuration. We only support configuration 1 */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*) eventData;
            if (configuredEventData->configurationValue == 1) {
                /* Update LED to show configured state */

                /* Register the CDC Device application event handler here.
                 * Note how the appData object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, APP_USBDeviceCDCEventHandler, (uintptr_t) & appData);

                /* Mark that the device is now configured */
                appData.isConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Switch LED to show suspended state */
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/*****************************************************
 * This function is called in every step of the
 * application state machine.
 *****************************************************/

bool APP_StateReset(void) {
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if (appData.isConfigured == false) {
        appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
        appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.isReadComplete = true;
        appData.isWriteComplete = true;
        retVal = true;
    } else {
        retVal = false;
    }

    return (retVal);
}

void initIMU() {
    ANSELBbits.ANSB2 = 0; //turn off B2 pin as analog input
    ANSELBbits.ANSB3 = 0; //turn off B3 pin as analog input

    i2c_master_setup(); //set up i2c

    //Set CTRL values by sequential write
    i2c_master_start(); //begin the start sequence
    i2c_master_send(SLAVE_ADDR << 1); //send slave address and left shift by 1 to indicate write with LSB 0
    i2c_master_send(CTRL1_ADDR); //send register address to CTRL1 register
    i2c_master_send(0x82); //send data to set CTRL1
    i2c_master_send(0x88); //send data to set CTRL2
    i2c_master_send(0x04); //send data to set CTRL3
    i2c_master_stop();
}

char whoAmI() {
    char who = 0;
    i2c_master_start(); //begin the start sequence
    i2c_master_send(SLAVE_ADDR << 1); //send slave address and left shift by 1 to indicate write with LSB 0
    i2c_master_send(WHO_AM_I_ADDR); //send WHO_AM_I register address
    i2c_master_restart();
    i2c_master_send(SLAVE_ADDR << 1 | 0x01); //send slave address and left shift by 1 and or 1 to indicate read with LSB 1
    who = i2c_master_recv(); //receive WHO_AM_I value
    i2c_master_ack(1);
    i2c_master_stop();

    return who;
}

void I2C_read_multiple(unsigned char slave_address, unsigned char reg, unsigned char * data, int length) { //to read values from accelerometer address
    int i = 0;
    i2c_master_start(); //begin the start sequence
    i2c_master_send(slave_address << 1); //send slave address and left shift by 1 to indicate write with LSB 0
    i2c_master_send(reg); //send register address
    i2c_master_restart();
    i2c_master_send(slave_address << 1 | 0x01); //send slave address and left shift by 1 and or 1 to indicate read with LSB 1
    for (i = 0; i < length - 1; i++) { //loop for number of data to read
        data[i] = i2c_master_recv();
        i2c_master_ack(0);
    }
    data[length - 1] = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
}

void display_char(unsigned short x, unsigned short y, unsigned short color1, unsigned short color2, char c) {
    int i = 0, j = 0;
    unsigned short dot = 0;
    for (i = 0; i < 5; i++) {
        for (j = 0; j < 8; j++) {
            if (x + i < 128 && y + j < 128) {
                dot = (ASCII[c - 0x20][i] >> j)&0x1;
                if (dot == 1) {
                    LCD_drawPixel(x + i, y + j, color1);
                } else if (dot == 0) {
                    LCD_drawPixel(x + i, y + j, color2);
                }
            }
        }
    }
}

void display_string(unsigned short xpos, unsigned short ypos, char* msg) {
    int counter = 0;
    while (msg[counter]) {
        display_char(xpos + 5 * counter, ypos, TEXT, BACKGROUND, msg[counter]);
        counter++;
    }
}

void draw_barX(unsigned short x1, unsigned short y1, unsigned short w, unsigned short colorA, unsigned short colorB, signed short len) {
    signed short xdirX = 0, ydirX = 0;
    if (len >= 0) { //check if direction is positive
        if ((len / CAL) < BAR_LENGTH) {
            for (xdirX = 0; xdirX < (len / CAL); xdirX++) { //print colorA for len
                for (ydirX = 0; ydirX < w; ydirX++) {
                    LCD_drawPixel(x1 - xdirX, y1 - ydirX, colorA);
                }
            }
            for (xdirX = (len / CAL); xdirX < BAR_LENGTH; xdirX++) { //print colorB for len to BAR_LENGTH
                for (ydirX = 0; ydirX < w; ydirX++) {
                    LCD_drawPixel(x1 - xdirX, y1 - ydirX, colorB);
                }
            }
            for (xdirX = 0; xdirX > (-BAR_LENGTH); xdirX--) { //print colorB for opposite direction
                for (ydirX = 0; ydirX < w; ydirX++) {
                    LCD_drawPixel(x1 - xdirX, y1 - ydirX, colorB);
                }
            }
        } else {
            for (xdirX = 0; xdirX < BAR_LENGTH; xdirX++) { //print full bar if out of range
                for (ydirX = 0; ydirX < w; ydirX++) {
                    LCD_drawPixel(x1 - xdirX, y1 - ydirX, colorA);
                }
            }
        }
    } else if (len < 0) { //case if negative length
        if ((len / CAL)>(-BAR_LENGTH)) { //check if range within BAR_LENGTH
            for (xdirX = 0; xdirX > (len / CAL); xdirX--) { //print colorA for len
                for (ydirX = 0; ydirX < w; ydirX++) {
                    LCD_drawPixel(x1 - xdirX, y1 - ydirX, colorA);
                }
            }
            for (xdirX = len / CAL; xdirX > (-BAR_LENGTH); xdirX--) { //print colorB for len to BAR_LENGTH
                for (ydirX = 0; ydirX < w; ydirX++) {
                    LCD_drawPixel(x1 - xdirX, y1 - ydirX, colorB);
                }
            }
            for (xdirX = 0; xdirX < BAR_LENGTH; xdirX++) { //print colorB for opposite direction
                for (ydirX = 0; ydirX < w; ydirX++) {
                    LCD_drawPixel(x1 - xdirX, y1 - ydirX, colorB);
                }
            }
        } else {
            for (xdirX = 0; xdirX > (-BAR_LENGTH); xdirX--) { //print full bar if out of range
                for (ydirX = 0; ydirX < w; ydirX++) {
                    LCD_drawPixel(x1 - xdirX, y1 - ydirX, colorA);
                }
            }
        }
    }
}

void draw_barY(unsigned short x2, unsigned short y2, unsigned short w, unsigned short colorA, unsigned short colorB, signed short len) {
    signed short xdirY = 0, ydirY = 0;
    if (len >= 0) {
        if ((len / CAL) < BAR_LENGTH) {
            for (ydirY = 0; ydirY < (len / CAL); ydirY++) {
                for (xdirY = 0; xdirY < w; xdirY++) {
                    LCD_drawPixel(x2 - xdirY, y2 - ydirY, colorA);
                }
            }
            for (ydirY = (len / CAL); ydirY < BAR_LENGTH; ydirY++) {
                for (xdirY = 0; xdirY < w; xdirY++) {
                    LCD_drawPixel(x2 - xdirY, y2 - ydirY, colorB);
                }
            }
            for (ydirY = 0; ydirY > (-BAR_LENGTH); ydirY--) {
                for (xdirY = 0; xdirY < w; xdirY++) {
                    LCD_drawPixel(x2 - xdirY, y2 - ydirY, colorB);
                }
            }
        } else {
            for (ydirY = 0; ydirY < BAR_LENGTH; ydirY++) {
                for (xdirY = 0; xdirY < w; xdirY++) {
                    LCD_drawPixel(x2 - xdirY, y2 - ydirY, colorA);
                }
            }
        }
    } else if (len < 0) {
        if ((len / CAL)>(-BAR_LENGTH)) {
            for (ydirY = 0; ydirY > (len / CAL); ydirY--) {
                for (xdirY = 0; xdirY < w; xdirY++) {
                    LCD_drawPixel(x2 - xdirY, y2 - ydirY, colorA);
                }
            }
            for (ydirY = len / CAL; ydirY > (-BAR_LENGTH); ydirY--) {
                for (xdirY = 0; xdirY < w; xdirY++) {
                    LCD_drawPixel(x2 - xdirY, y2 - ydirY, colorB);
                }
            }
            for (ydirY = 0; ydirY < BAR_LENGTH; ydirY++) {
                for (xdirY = 0; xdirY < w; xdirY++) {
                    LCD_drawPixel(x2 - xdirY, y2 - ydirY, colorB);
                }
            }
        } else {
            for (ydirY = 0; ydirY > (-BAR_LENGTH); ydirY--) {
                for (xdirY = 0; xdirY < w; xdirY++) {
                    LCD_drawPixel(x2 - xdirY, y2 - ydirY, colorA);
                }
            }
        }
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )
  Remarks:
    See prototype in app.h.
 */

void APP_Initialize(void) {
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    /* Device Layer Handle  */
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;

    /* Device configured status */
    appData.isConfigured = false;

    /* Initial get line coding state */
    appData.getLineCodingData.dwDTERate = 9600;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bDataBits = 8;

    /* Read Transfer Handle */
    appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Write Transfer Handle */
    appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Intialize the read complete flag */
    appData.isReadComplete = true;

    /*Initialize the write complete flag*/
    appData.isWriteComplete = true;

    /* Reset other flags */
    appData.sofEventHasOccurred = false;
    //appData.isSwitchPressed = false;

    /* Set up the read buffer */
    appData.readBuffer = &readBuffer[0];

    /*Initialise IMU*/
    TRISBbits.TRISB4 = 1; //Set pin RB4 as an input pin
    TRISAbits.TRISA4 = 0; //Set pin RA4 as an output pin
    LATAbits.LATA4 = 1; //Initialise LED to be HIGH
    SPI1_init();
    LCD_init();
    LCD_clearScreen(GREEN);
    initIMU();

    startTime = _CP0_GET_COUNT();
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )
  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {
    /* Update the application state machine based
     * on the current state */

    switch (appData.state) {
        case APP_STATE_INIT:


            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE);

            if (appData.deviceHandle != USB_DEVICE_HANDLE_INVALID) {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle, APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            } else {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }
            break;


        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device was configured */
            if (appData.isConfigured) {
                /* If the device is configured then lets start reading */
                appData.state = APP_STATE_SCHEDULE_READ;
            }
            break;

        case APP_STATE_SCHEDULE_READ:

            if (APP_StateReset()) {
                break;
            }

            /* If a read is complete, then schedule a read
             * else wait for the current read to complete */

            appData.state = APP_STATE_WAIT_FOR_READ_COMPLETE;
            if (appData.isReadComplete == true) {
                appData.isReadComplete = false;
                appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

                USB_DEVICE_CDC_Read(USB_DEVICE_CDC_INDEX_0,
                        &appData.readTransferHandle, appData.readBuffer,
                        APP_READ_BUFFER_SIZE);

                if (appData.readTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID) {
                    appData.state = APP_STATE_ERROR;
                    break;
                }
            }

            break;

        case APP_STATE_WAIT_FOR_READ_COMPLETE:
        case APP_STATE_CHECK_TIMER:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was received or a switch was pressed.
             * The isReadComplete flag gets updated in the CDC event handler. */

            if (appData.isReadComplete || _CP0_GET_COUNT() - startTime > (48000000 / 2 / 100)) {
                appData.state = APP_STATE_SCHEDULE_WRITE;
            }

            break;


        case APP_STATE_SCHEDULE_WRITE:

            if (APP_StateReset()) {
                break;
            }

            /* Setup the write */

            appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
            appData.isWriteComplete = false;
            appData.state = APP_STATE_WAIT_FOR_WRITE_COMPLETE;

            dataOut[0] = 0;

            if (appData.isReadComplete) {
                if (appData.readBuffer[0] == 'r') {
                    printData = 1;
                    USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                            &appData.writeTransferHandle, dataOut, 1,
                            USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                } else {
                    len = sprintf(msg, "Error\r\n");
                    USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                            &appData.writeTransferHandle,
                            msg, len,
                            USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                }
            } else if (printData == 1) {
                if (sampleIndex < SAMPLE_NO) {
                    I2C_read_multiple(SLAVE_ADDR, OUTZ_L_XL_ADDR, data, 2);
                    for (j = 0; j < 1; j++) {
                        data_array[j] = (data[2 * j + 1] << 8) | data[2 * j];
                    }
                    mafCalc[k] = data_array[0];
                    mafSum = 0;
                    firAve = 0;
                    coInd = k;
                    for (n = 0; n < MAX_LENGTH; n++) {
                        mafSum = mafSum + mafCalc[n];
                        firAve = firAve + firCoeff[n]*mafCalc[coInd];
                        if (coInd<MAX_LENGTH-1){
                            coInd++;
                        }
                        else{
                            coInd = 0;
                        }
                    }
                    mafAve = mafSum / MAX_LENGTH;
                    k++;
                    if (k == MAX_LENGTH) {
                        k = 0;
                    }
                    iirAve = ALPHA*prevAve + BETA*data_array[0];
                    prevAve = iirAve;
                    
                    len = sprintf(msg, "%d %d %d %d\r\n", sampleIndex, mafAve, (int) iirAve, (int) firAve);
                    sampleIndex++;
                    USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                            &appData.writeTransferHandle,
                            msg, len,
                            USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                } else {
                    sampleIndex = 0;
                    printData = 0;
                    USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                            &appData.writeTransferHandle, dataOut, 1,
                            USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);

                }
            } else {
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &appData.writeTransferHandle, dataOut, 1,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                startTime = _CP0_GET_COUNT();
            }
            break;

        case APP_STATE_WAIT_FOR_WRITE_COMPLETE:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was sent. The isWriteComplete
             * flag gets updated in the CDC event handler */

            if (appData.isWriteComplete == true) {
                appData.state = APP_STATE_SCHEDULE_READ;
            }

            break;

        case APP_STATE_ERROR:
            break;
        default:
            break;
    }
}



/*******************************************************************************
 End of File
 */