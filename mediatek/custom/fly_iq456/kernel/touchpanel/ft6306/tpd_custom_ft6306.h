#ifndef TOUCHPANEL_H__
#define TOUCHPANEL_H__

/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE
//#define TPD_TYPE_RESISTIVE
#define TPD_I2C_NUMBER           1
#define TPD_WAKEUP_TRIAL         60
#define TPD_WAKEUP_DELAY         100

#define TPD_DELAY                (2*HZ/100)
#define TPD_RES_X                480
#define TPD_RES_Y                854
#define TPD_CALIBRATION_MATRIX  {962,0,0,0,1600,0,0,0};

//#define TPD_HAVE_CALIBRATION
#define TPD_HAVE_TREMBLE_ELIMINATION

//#define TPD_HAVE_POWER_ON_OFF
#define PRESSURE_FACTOR 10

#define TPD_HAVE_BUTTON

#define TPD_BUTTON_HEIGHT       50
#define TPD_KEY_COUNT           3

// Выставить в таком порядке, в котором кнопки расположены у вас
#define TPD_KEYS                {KEY_MENU, KEY_HOMEPAGE, KEY_BACK}

// В программе DeviceInfoMTK в поле TPD увидите значения, у меня это
// 0x01:139:55:900:110:50:0x01:172:240:900:110:50:0x01:158:424:900:110:50
// значит мои значения такие 
// 55:900:110:50
// 240:900:110:50
// 424:900:110:50
#define TPD_KEYS_DIM            {{55,900,110,TPD_BUTTON_HEIGHT},{240,900,110,TPD_BUTTON_HEIGHT},{424,900,110,TPD_BUTTON_HEIGHT}}


#endif /* TOUCHPANEL_H__ */
