
#include "wiced_platform.h"
#include "wiced_timer.h"
#include "wiced_hal_i2c.h"
#include "wiced_bt_trace.h"

#define POLL_TIMER 1 /*1s timer for reading from sensor*/
#define CLK_FREQ 24000 /*Clock frequency in KHz*/

static wiced_timer_t ma12070p_timer;

void deinit_ma12070p(void);
static void ma12070p_cb(uint32_t arg);

void init_ma12070p(void)
{

    wiced_hal_i2c_init();

    wiced_hal_i2c_set_speed(I2CM_SPEED_100KHZ);
    WICED_BT_TRACE("Current I2C speed: %d KHz\n", CLK_FREQ/wiced_hal_i2c_get_speed());

    wiced_init_timer( &ma12070p_timer, &ma12070p_cb, 0, WICED_SECONDS_PERIODIC_TIMER );
    if ( WICED_SUCCESS == wiced_init_timer( &ma12070p_timer, &ma12070p_cb, 0, WICED_SECONDS_PERIODIC_TIMER )) {
        if ( WICED_SUCCESS != wiced_start_timer( &ma12070p_timer, POLL_TIMER )) {
            WICED_BT_TRACE( "Seconds Timer Error\n\r" );
        }
    }

}

void deinit_ma12070p(void)
{
    wiced_stop_timer( &ma12070p_timer );                                        // stop timer
    wiced_deinit_timer( &ma12070p_timer );                                      // deinit callback
}

void ma12070p_cb(uint32_t arg)
{

    UINT8 status = 0xFF;
    UINT8 wbuf[] = {0x35,0x00,0x73};    // reg, dat0, dat1
    UINT8 rbuf[] = {0x00,0x00};

    status = wiced_hal_i2c_combined_read((UINT8*)&rbuf, sizeof(rbuf), &wbuf[0], sizeof(UINT8), 0x20);

    if( status == I2CM_SUCCESS) {

        WICED_BT_TRACE("I2C read %Xh=0x%X\r\n", wbuf[0], rbuf[0]);
        WICED_BT_TRACE("I2C read %Xh=0x%X\r\n", wbuf[0]+1, rbuf[1]);
        if( (rbuf[0]==wbuf[1]) && (rbuf[1]==wbuf[2]) ){                                 // check init data
            ;
        }else{
            wiced_hal_i2c_write((UINT8*)&wbuf, sizeof(wbuf), 0x20);                     // write init data
        }

    }else if(I2CM_OP_FAILED == status) {
        WICED_BT_TRACE("I2C comboread operation failed\r\n");
//        deinit_ma12070p();
//        init_ma12070p();
    }else if(I2CM_BUSY == status) {
        WICED_BT_TRACE("I2C busy\r\n");
    }else{
        WICED_BT_TRACE("Unknown status from I2C\r\n");
    }

}
