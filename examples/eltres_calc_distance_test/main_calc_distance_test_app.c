// ==========================================================================
/*!
* @file     main_calc_distance_test_app.c
* @brief    test application for calc_distance
* @date     2023/12/12
*
* Copyright 2021 - 2023 Sony Semiconductor Solutions Corporation
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
#include "main_calc_distance_test_app.h"
#include "calc_distance.h"
#include "calc_distance_port.h"

// Sample application name definition
#define SAMPLE_APP_NAME   "main_calc_distance_test_app"

// Sample application version definition
#define SAMPLE_APP_VER    "0.0.1"

// ===========================================================================
//! calc distance test application main function
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
int main_calc_distance_test_app(void){
    int fail_count = 0;
    printf("-- result json start ---\r\n");
    printf("{\r\n");
    fail_count += calc_distance_port_test_main();
    printf(",\r\n");
    fail_count += calc_distance_core_test_main();
    printf(",\r\n  \"total_fail_count\": %d\r\n", fail_count);
    printf("}\r\n");
    printf("-- result json end ---\r\n");
    return 0;
}

