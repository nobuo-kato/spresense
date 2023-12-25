// ==========================================================================
/*!
* @file     main_calc_distance_sample_app.c
* @brief    calc distancd sample application
* @date     2023/12/13
*
* Copyright 2021, 2022 Sony Semiconductor Solutions Corporation
* 
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
* 
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation and/or
* other materials provided with the distribution.
* 
* 3. Neither the name of Sony Semiconductor Solutions Corporation nor the names of
* its contributors may be used to endorse or promote products derived from this
* software without specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
* OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/
// =========================================================================

/* Includes ------------------------------------------------------------------*/
/*Host Microcomputer dependence include BEGIN */

#include <nuttx/config.h>
#include <nuttx/timers/rtc.h>
#include <arch/board/board.h>
#include <arch/chip/pin.h>

/*Host Microcomputer dependence include END   */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <assert.h>
#include "CXM150x_APITypeDef.h"
#include "CXM150x_GNSS.h"
#include "CXM150x_SYS.h"
#include "CXM150x_TIME.h"
#include "CXM150x_TX.h"
#include "CXM150x_LIB.h"
#include "CXM150x_Utility.h"
#include "CXM150x_Port.h"
#include "main_calc_distance_sample_app.h"
#include "calc_distance.h"

// Sample application name definition
#define SAMPLE_APP_NAME   "main_calc_distance_sample_app"
// Sample application version definition
#define SAMPLE_APP_VER    "3.0.3"

#define EEPROM_P1_POW_MODE_OFFSET (0x0450)
#define EEPROM_P2_POW_MODE_OFFSET (0x0550)
#define EEPROM_EVT_POW_MODE_OFFSET (0x0650)

#if defined(CONFIG_EXTERNALS_ELTRES_SPEXEL)

// INT_BUTTON pin assignment
#define INT_BUTTON1_PIN   PIN_SPI4_SCK
#define INT_BUTTON2_PIN   PIN_SPI4_MISO
#define INT_BUTTON3_PIN   PIN_SPI4_MOSI
#define INT_BUTTON4_PIN   PIN_SPI4_CS_X
#define POWER_SDCARD      PMIC_GPO(5)

#endif

// Leap second correction (Leap second as of Dec 2023)
#define GNSS_DATETIME_ADJUST_LEAP_SECOND        (18)

// Flag ON / OFF definition
typedef enum {
    FLAG_OFF = 0,
    FLAG_ON
}FlagOnOff;

// buffer for event information
static CXM150xSysState g_sys_stt_info;
static CXM150xGNSSState g_gnss_stt_info;
static CXM150xFATALMessage g_fatalmessage_info;
static CXM150xEventBufferOverflow g_buffer_overflow_info;

// Push button interrupt flag
static uint8_t g_push_btn_flag = FLAG_ON;

// GNSS is avairable if CXM150x status is "EPM_FILL" or later state
static uint32_t g_gnss_ready_flg = FLAG_OFF;

// ===========================================================================
//! Callback function when a system state event occurs
/*!
 *
 * @param [in] info: Response data structure
 * @param [in] id: Callback ID
 * @param [out] none
 * @par Global variable
 *        [in] g_sys_stt_info: Response data structure
 *        [out] g_gnss_ready_flg: GNSS ready flag
 *        [out] g_tx_comlete_flg: transmission completion flag
 *        [out] g_ev_tx_complete_flag: Event transmission completion flag
 *        [out] g_gnss_timeout_flag: GNSS acquisition timeout flag
 *        [out] g_gnss_backup_done_flag: GNSS backup completion wait flag
 * @return none
*/
// ===========================================================================
static void sys_stt_event_callback(void *info,uint32_t id){

    if(g_sys_stt_info == SYS_STT_IDLE){
        printf("sys_stt_event_callback:code=%d(IDLE)\r\n",g_sys_stt_info);
    } else if(g_sys_stt_info == SYS_STT_FETCHING_TIME){
        printf("sys_stt_event_callback:code=%d(FETCHING_TIME)\r\n",g_sys_stt_info);
    } else if(g_sys_stt_info == SYS_STT_WAIT_FETCHING_TIME){
        printf("sys_stt_event_callback:code=%d(WAIT_FETCHING_TIME)\r\n",g_sys_stt_info);
    } else if(g_sys_stt_info == SYS_STT_EPM_FILL){
        printf("sys_stt_event_callback:code=%d(EPM_FILL)\r\n",g_sys_stt_info);
        g_gnss_ready_flg = FLAG_ON;
    } else if(g_sys_stt_info == SYS_STT_WAIT_TX_PREPARE){
        printf("sys_stt_event_callback:code=%d(WAIT_TX_PREPARE)\r\n",g_sys_stt_info);
    } else if(g_sys_stt_info == SYS_STT_AF_TX_PREPARE){
        printf("sys_stt_event_callback:code=%d(AF_TX_PREPARE)\r\n",g_sys_stt_info);
    } else if(g_sys_stt_info == SYS_STT_AF_WAIT_TX_START){
        printf("sys_stt_event_callback:code=%d(AF_WAIT_TX_START)\r\n",g_sys_stt_info);
    } else if(g_sys_stt_info == SYS_STT_AF_TX_PROGRESS){
        printf("sys_stt_event_callback:code=%d(AF_TX_PROGRESS)\r\n",g_sys_stt_info);
    } else if(g_sys_stt_info == SYS_STT_DF_TX_PREPARE){
        printf("sys_stt_event_callback:code=%d(DF_TX_PREPARE)\r\n",g_sys_stt_info);
    } else if(g_sys_stt_info == SYS_STT_DF_WAIT_TX_START){
        printf("sys_stt_event_callback:code=%d(DF_WAIT_TX_START)\r\n",g_sys_stt_info);
    } else if(g_sys_stt_info == SYS_STT_DF_TX_PROGRESS){
        printf("sys_stt_event_callback:code=%d(DF_TX_PROGRESS)\r\n",g_sys_stt_info);
    } else if(g_sys_stt_info == SYS_STT_EV_TX_COMPLETE){
        printf("sys_stt_event_callback:code=%d(SYS_STT_EV_TX_COMPLETE)\r\n",g_sys_stt_info);
    } else if(g_sys_stt_info == SYS_STT_GNSS_BACKUP){
        printf("sys_stt_event_callback:code=%d(SYS_STT_GNSS_BACKUP)\r\n",g_sys_stt_info);
    } else if(g_sys_stt_info == SYS_STT_GNSS_BACKUP_DONE){
        printf("sys_stt_event_callback:code=%d(SYS_STT_GNSS_BACKUP_DONE)\r\n",g_sys_stt_info);
    } else{
        printf("sys_stt_event_callback:code=%d(PURSE_ERROR)\r\n",g_sys_stt_info);
    }

}

// ===========================================================================
//! Callback function when a GNSS operation state event occurs
/*!
 *
 * @param [in] info: GNSS operation state event information
 * @param [in] id: Callback ID
 * @param [out] none
 * @par Global variable
 *        [in] g_gnss_stt_info: GNSS operation state event information buffer
 *        [out] g_gnss_sleep_flag: GNSS Sleep flag
 * @return none
*/
// ===========================================================================
static void gnss_stt_event_callback(void *info,uint32_t id){
    printf("gnss_stt_event_callback:code=0x%02x\r\n", g_gnss_stt_info);
    if(g_gnss_stt_info & 0x80){   // Sleeping
        printf("*** GNSS_SLEEP ***\r\n");
    }
}

// ===========================================================================
//! Callback function when FATAL message event occurs
/*!
 *
 * @param [in] info: Abnormal event information
 * @param [in] id: Callback ID
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 * @return none
*/
// ===========================================================================
static void fatal_message_event_callback(void *info,uint32_t id){
    CXM150xFATALMessage *fatal_info = (CXM150xFATALMessage*)info;
    printf("FATAL Message Event: %s\r\n",fatal_info->m_str);
}

// ===========================================================================
//! Event buffer overflow callback function
/*!
 *
 * @param [in] info: Event data information
 * @param [in] id: Callback ID
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 * @return none
*/
// ===========================================================================
static void event_buffer_overflow_callback(void *info,uint32_t id){
    printf("event buffer overflow\r\n");
}

// ===========================================================================
//! Display sample app and API version
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 * @return none
*/
// ===========================================================================
static void disp_version(void){
    CmdResGetCXM150xAPIVersion api_ver_inf;
    get_CXM150x_api_version(NULL,&api_ver_inf,NULL);
    printf("%s:Ver.%s, API:Ver.%s\r\n",SAMPLE_APP_NAME,SAMPLE_APP_VER,api_ver_inf.m_version);
}

// ===========================================================================
//! Push button interrupt callback function
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] g_push_btn_flag: Push button interrupt flag
 *
 * @return none
*/
// ===========================================================================
#if defined(CONFIG_EXTERNALS_ELTRES_SPEXEL)
static void push_btn_CXM150x(void){
    g_push_btn_flag=
    !( board_gpio_read(INT_BUTTON1_PIN)&&
    board_gpio_read(INT_BUTTON2_PIN)&&
    board_gpio_read(INT_BUTTON3_PIN)&&
    board_gpio_read(INT_BUTTON4_PIN));
    printf("================== push_btn_callback %d ===========\r\n", g_push_btn_flag);
}
#endif

// ===========================================================================
//! Wait until GNSS is available
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] g_gnss_ready_flg: GNSS ready flag
 *        [out] none
 * @return none
*/
// ===========================================================================
static void wait_gps_data(void){
    if(g_gnss_ready_flg == FLAG_OFF){
        printf("-wait GNSS ready-\r\n");
        // Set system state event to ON
        register_CXM150x_sys_state_event(&g_sys_stt_info,sys_stt_event_callback);
        CmdResSetCXM150xSysStateEvent res_sys_stt;
        memset(&res_sys_stt,0,sizeof(res_sys_stt));
        set_CXM150x_sys_state_event(EVENT_ON,&res_sys_stt,NULL);
        {
            //GNSS EVENT ON
            memset(&g_gnss_stt_info,'\0',sizeof(g_gnss_stt_info));
            register_CXM150x_GNSS_state_event(&g_gnss_stt_info, gnss_stt_event_callback);
            CmdResSetCXM150xGNSSStateEvent res_set_gnss_stt;
            memset(&res_set_gnss_stt,0,sizeof(res_set_gnss_stt));
            set_CXM150x_GNSS_state_event(EVENT_ON,&res_set_gnss_stt, NULL);
            
        }
        while(g_gnss_ready_flg == FLAG_OFF){
            analyse_CXM150x_Rx();
        }
        
        memset(&res_sys_stt,0,sizeof(res_sys_stt));
        set_CXM150x_sys_state_event(EVENT_OFF,&res_sys_stt,NULL);
        printf("-GNSS ready-\r\n");
    }
}

// ===========================================================================
//! Current time acquisition function
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 * @return current time
*/
// ===========================================================================
static uint32_t get_current_tm(void){
    // Get current time
    CmdResGetCXM150xCurrentGNSSTime res;
    while(1){
        get_CXM150x_current_GNSS_time(NULL,&res,NULL);
        if(res.m_str[0] != '\0'){
            break;
        } else {
            sleep(1);
        }
    }
    
    uint32_t sec = 0;
    conv_CXM150x_GNSSTime_to_second(res.m_str,&sec);
    
    // Convert the number of seconds elapsed since GPS reference date and time obtained by above API to the number of seconds elapsed since 00:00:00 on January 1, 1900
    // Convert by adding the number of seconds elapsed from 00:00:00 on January 1, 1900 to 00:00:00 on January 6, 1980, which is the base date of GPS time
    struct tm base_time;
    base_time.tm_sec = GPS_FORMAT_BASE_TIME_SEC;
    base_time.tm_min = GPS_FORMAT_BASE_TIME_MIN;
    base_time.tm_hour = GPS_FORMAT_BASE_TIME_HOUR;
    base_time.tm_mday = GPS_FORMAT_BASE_TIME_MDAY;
    base_time.tm_mon = GPS_FORMAT_BASE_TIME_MON;
    base_time.tm_year = GPS_FORMAT_BASE_TIME_YEAR;
    base_time.tm_wday = GPS_FORMAT_BASE_TIME_WDAY;
    base_time.tm_yday = GPS_FORMAT_BASE_TIME_YDAY;
    base_time.tm_isdst = GPS_FORMAT_BASE_TIME_ISDST;
    sec += mktime(&base_time);

    // This time format is GNSS time so substract leap second to convert it to UTC time format
    sec -= GNSS_DATETIME_ADJUST_LEAP_SECOND;

    return sec;
}

// ===========================================================================
//! Set RTC timer
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 * @return current time
*/
// ===========================================================================
static void set_rtc_time(void){
    uint32_t sec = get_current_tm();

    struct timespec ts;
    ts.tv_sec = sec;
    ts.tv_nsec = 0;
    clock_settime(CLOCK_REALTIME, &ts);

    struct tm s_tm;
    localtime_r(&ts.tv_sec, &s_tm);
    printf("local time:%02d:%02d.%02d(utc)\r\n",s_tm.tm_hour,s_tm.tm_min,s_tm.tm_sec);
    printf("RTC init:%04d-%02d-%02d %02d:%02d:%02d\r\n",
           s_tm.tm_year + 1900, s_tm.tm_mon + 1, s_tm.tm_mday, s_tm.tm_hour, s_tm.tm_min, s_tm.tm_sec);
    
}

// ===========================================================================
//! check message count
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] none
 *        [out] none
 * @return FLAG_ON if message exist
*/
// ===========================================================================
static uint32_t check_message_count(void){
    // Get power supply status
    CmdResGetCXM150xPower pwstate;
    get_CXM150x_power(NULL,&pwstate,NULL);
    
    // resume if the power is off
    if(pwstate.m_num == CXM150x_POWER_OFF){
        return FLAG_OFF;
    } else {
        if(get_CXM150x_Rx_message_count()>0){
            return FLAG_ON;
        } else {
            return FLAG_OFF;
        }
    }
}

// ===========================================================================
//! LPWA sample application main function
/*!
 *
 * @param [in] none
 * @param [out] none
 * @par Global variable
 *        [in] g_push_btn_flag: Push button interrupt flag
 *        [in] g_int1_callback_flg:  INT_OUT1 interrupt flag 
 *        [in] g_ev_tx_complete_flag: Event transmission completion flag
 *        [in] g_sys_stt_info: system state event information
 *        [in] g_buffer_overflow_info: Response data structure
 *        [out] g_pow_enable_remain_offset: CXM150x power ON offset time
 *        [out] g_periodec_enable: Periodic enable flag
 *        [out] g_profile_event_enable: Event transmission enable flag
 * @return exit code
*/
// ===========================================================================
int main_calc_distance_sample_app(void){
    // Display version information
    disp_version();
    printf_info("This message is used printf_info()\r\n");

    // FATAL message event callback setting
    register_CXM150x_FATAL_message_event(&g_fatalmessage_info,fatal_message_event_callback);
    register_CXM150x_event_buffer_overflow(&g_buffer_overflow_info,event_buffer_overflow_callback);

    // Power ON and set normal mode
    CmdResSetCXM150xPower res_set_power;
    memset(&res_set_power,0,sizeof(res_set_power));
    set_CXM150x_power(CXM150x_POWER_ON,&res_set_power,NULL);

    CmdResSetCXM150xMode res_set_mode;
    memset(&res_set_mode,0,sizeof(res_set_mode));
    set_CXM150x_mode(CXM150x_MODE_NORMAL,&res_set_mode,NULL);

#if defined(CONFIG_EXTERNALS_ELTRES_SPEXEL)
    // Button interrupt setting
    board_gpio_config(INT_BUTTON1_PIN, 0, true, false, PIN_PULLUP);
    board_gpio_intconfig(INT_BUTTON1_PIN, INT_BOTH_EDGE, true, (xcpt_t)push_btn_CXM150x);
    board_gpio_int(INT_BUTTON1_PIN, true);

    board_gpio_config(INT_BUTTON2_PIN, 0, true, false, PIN_PULLUP);
    board_gpio_intconfig(INT_BUTTON2_PIN, INT_BOTH_EDGE, true, (xcpt_t)push_btn_CXM150x);
    board_gpio_int(INT_BUTTON2_PIN, true);

    board_gpio_config(INT_BUTTON3_PIN, 0, true, false, PIN_PULLUP);
    board_gpio_intconfig(INT_BUTTON3_PIN, INT_BOTH_EDGE, true, (xcpt_t)push_btn_CXM150x);
    board_gpio_int(INT_BUTTON3_PIN, true);

    board_gpio_config(INT_BUTTON4_PIN, 0, true, false, PIN_PULLUP);
    board_gpio_intconfig(INT_BUTTON4_PIN, INT_BOTH_EDGE, true, (xcpt_t)push_btn_CXM150x);
    board_gpio_int(INT_BUTTON4_PIN, true);
#endif

    // Get various EEPROM settings
    CmdResGetCXM150xEEPROMData eep_data;

    get_CXM150x_EEPROM_data(EEPROM_P1_POW_MODE_OFFSET,&eep_data,NULL);
    printf_info("PowMode for periodic 1 of EEPROM is %ld. %s\r\n" ,eep_data.m_num, eep_data.m_num!=0?"0 is recommended.":"");

    get_CXM150x_EEPROM_data(EEPROM_P2_POW_MODE_OFFSET,&eep_data,NULL);
    printf_info("PowMode for periodic 2 of EEPROM is %ld. %s\r\n" ,eep_data.m_num, eep_data.m_num!=0?"0 is recommended.":"");

    // Set system state event to ON
    register_CXM150x_sys_state_event(&g_sys_stt_info,sys_stt_event_callback);
    CmdResSetCXM150xSysStateEvent res_sys_stt;
    memset(&res_sys_stt,0,sizeof(res_sys_stt));
    set_CXM150x_sys_state_event(EVENT_ON,&res_sys_stt,NULL);

    //GNSS EVENT ON
    memset(&g_gnss_stt_info,'\0',sizeof(g_gnss_stt_info));
    register_CXM150x_GNSS_state_event(&g_gnss_stt_info, gnss_stt_event_callback);
    CmdResSetCXM150xGNSSStateEvent res_set_gnss_stt;
    memset(&res_set_gnss_stt,0,sizeof(res_set_gnss_stt));
    set_CXM150x_GNSS_state_event(EVENT_ON,&res_set_gnss_stt, NULL);

    // initialize calc distance.    
    init_calc_distance((void *)0);

    // GPS data acquisition
    wait_gps_data();

    // RTC settings
    set_rtc_time();

    open_logfile();
    static int btn_t0=0;
    {
        struct timespec ts={0};
        clock_gettime(CLOCK_REALTIME, &ts);
        btn_t0=ts.tv_sec;
    }

    while(1){
        struct timespec ts={0};
        int ret = clock_gettime(CLOCK_REALTIME, &ts);

        analyse_CXM150x_Rx();
        if(g_push_btn_flag == FLAG_OFF && check_message_count() == FLAG_OFF){
            // TBD: Implement sleep function
            sleep(1);
            continue;
        }
        while(get_CXM150x_Rx_message_count()>0){
            analyse_CXM150x_Rx();
        }

        if(g_push_btn_flag == FLAG_ON){
            if(!ret){
                set_led(ts.tv_sec-btn_t0);
                if(ts.tv_sec - btn_t0>5){
                    logging(0);
                    set_led(5);
                }
            }
        } else {
            if(!ret){
                btn_t0=ts.tv_sec;
            }
        }

    }
}

