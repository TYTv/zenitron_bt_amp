
#include "wiced_bt_cfg.h"
#include "wiced_bt_a2dp_sink.h"
#include "wiced_bt_audio.h"
#include "a2dp_cfg.h"
#include "wiced_bt_trace.h"
#include "wiced_memory.h"
#include "wiced_hal_nvram.h"
#include "wiced_platform.h"
#include "a2dp_sink.h"

#define WICED_HS_EIR_BUF_MAX_SIZE               264

uint8_t pincode[4]                         = { 0x30, 0x30, 0x30, 0x30 };

/*****************************************************************************
 *   codec and audio tuning configurations
 ****************************************************************************/
/*  Recommended max_bitpool for high quality audio */
#define BT_AUDIO_A2DP_SBC_MAX_BITPOOL   53

/* Array of decoder capabilities information. */
wiced_bt_a2dp_codec_info_t bt_audio_codec_capabilities[] =
{
    {
        .codec_id = WICED_BT_A2DP_CODEC_SBC,
        .cie =
        {
            .sbc =
            {
                (A2D_SBC_IE_SAMP_FREQ_44 | A2D_SBC_IE_SAMP_FREQ_48),    /* samp_freq */
                (A2D_SBC_IE_CH_MD_MONO   | A2D_SBC_IE_CH_MD_STEREO |
                 A2D_SBC_IE_CH_MD_JOINT  | A2D_SBC_IE_CH_MD_DUAL),      /* ch_mode */
                (A2D_SBC_IE_BLOCKS_16    | A2D_SBC_IE_BLOCKS_12 |
                 A2D_SBC_IE_BLOCKS_8     | A2D_SBC_IE_BLOCKS_4),        /* block_len */
                (A2D_SBC_IE_SUBBAND_4    | A2D_SBC_IE_SUBBAND_8),       /* num_subbands */
                (A2D_SBC_IE_ALLOC_MD_L   | A2D_SBC_IE_ALLOC_MD_S),      /* alloc_mthd */
                BT_AUDIO_A2DP_SBC_MAX_BITPOOL,                          /* max_bitpool for high quality audio */
                A2D_SBC_IE_MIN_BITPOOL                                  /* min_bitpool */
            }
        }
    },

#ifdef A2DP_SINK_AAC_ENABLED
    {
        .codec_id = WICED_BT_A2DP_CODEC_M24,
        .cie =
        {
            .m24 =
            {
                (A2D_M24_IE_OBJ_MSK),                                   /* obj_type */
                (A2D_M24_IE_SAMP_FREQ_44 | A2D_M24_IE_SAMP_FREQ_48),    /*samp_freq */
                (A2D_M24_IE_CHNL_MSK),                                  /* chnl */
                (A2D_M24_IE_VBR_MSK),                                   /* b7: VBR */
                (A2D_M24_IE_BITRATE_MSK)                                /* bitrate - b7-b0 of octect 3, all of octect4, 5*/
            }
        }
    }
#endif
};

/** A2DP sink configuration data */
wiced_bt_a2dp_config_data_t bt_audio_config =
{
#ifdef A2DP_SINK_ENABLE_CONTENT_PROTECTION
    .feature_mask = WICED_BT_A2DP_SINK_FEAT_PROTECT | WICED_BT_A2DP_SINK_FEAT_DELAY_RPT,    /* feature mask */
#else
    .feature_mask = WICED_BT_A2DP_SINK_FEAT_DELAY_RPT,                                      /* feature mask */
#endif
    .codec_capabilities =
    {
        .count = sizeof(bt_audio_codec_capabilities) / sizeof(bt_audio_codec_capabilities[0]),
        .info  = bt_audio_codec_capabilities,                                           /* codec configuration */
    },
    .p_param =
    {
        .buf_depth_ms                   = 300,                                          /* in msec */
        .start_buf_depth                = 50,                                           /* start playback percentage of the buffer depth */
        .target_buf_depth               = 50,                                           /* target level percentage of the buffer depth */
        .overrun_control                = WICED_BT_A2DP_SINK_OVERRUN_CONTROL_FLUSH_DATA,/* overrun flow control flag */
        .adj_ppm_max                    = +300,                                         /* Max PPM adjustment value */
        .adj_ppm_min                    = -300,                                         /* Min PPM adjustment value */
        .adj_ppb_per_msec               = 200,                                          /* PPM adjustment per milli second */
        .lvl_correction_threshold_high  = +2000,                                        /* Level correction threshold high value */
        .lvl_correction_threshold_low   = -2000,                                        /* Level correction threshold low value */
        .adj_proportional_gain          = 20,                                           /* Proportional component of total PPM adjustment */
        .adj_integral_gain              = 2,                                            /* Integral component of total PPM adjustment */
    },
    .ext_codec =
    {
#if ( ( defined(CYW20719B1) || defined(CYW20721B1) ) && (WICED_A2DP_EXT_CODEC == WICED_TRUE) )
        .codec_id        = WICED_AUDIO_CODEC_AAC_DEC,
        .codec_functions = &AAC_codec_function_table,
#else
        .codec_id        = WICED_AUDIO_CODEC_NONE,
        .codec_functions = 0,
#endif
    },
};

/**  Audio buffer configuration configuration */
const wiced_bt_audio_config_buffer_t a2dp_sink_audio_buf_config = {
    .role                       =   WICED_AUDIO_SINK_ROLE,
    .audio_tx_buffer_size       =   0,
#if ( ( defined(CYW20719B1) || defined(CYW20721B1) ) && (WICED_A2DP_EXT_CODEC == WICED_TRUE) )
    .audio_codec_buffer_size    =   0xD800
#else
    .audio_codec_buffer_size    =   0x5000
#endif
};








/*
 *  Prepare extended inquiry response data.  Current version publishes audio sink
 *  services.
 */
void a2dp_sink_write_eir( void )
{
    uint8_t *pBuf;
    uint8_t *p;
    uint8_t length;

    pBuf = (uint8_t*)wiced_bt_get_buffer( WICED_HS_EIR_BUF_MAX_SIZE );
    WICED_BT_TRACE( "a2dp_sink_write_eir %x\n", pBuf );

    if ( !pBuf )
    {
        return;
    }
    p = pBuf;

    length = strlen( (char *)wiced_bt_cfg_settings.device_name );

    *p++ = length + 1;
    *p++ = BT_EIR_COMPLETE_LOCAL_NAME_TYPE;   // EIR type full name
    memcpy( p, wiced_bt_cfg_settings.device_name, length );
    p += length;
    *p++ = ( 1 * 2 ) + 1;     // length of services + 1
    *p++ =   BT_EIR_COMPLETE_16BITS_UUID_TYPE;            // EIR type full list of 16 bit service UUIDs
    *p++ =   UUID_SERVCLASS_AUDIO_SINK        & 0xff;
    *p++ = ( UUID_SERVCLASS_AUDIO_SINK >> 8 ) & 0xff;
    *p++ = 0;

    // print EIR data
    wiced_bt_trace_array( "EIR :", ( uint8_t* )( pBuf+1 ), MIN( p-( uint8_t* )pBuf,100 ) );
    wiced_bt_dev_write_eir( pBuf, (uint16_t)(p - pBuf) );

    return;
}









/*
 * Write NVRAM function is called to store information in the NVRAM.
 */
int a2dp_sink_write_nvram( int nvram_id, int data_len, void *p_data)
{
    wiced_result_t  result;
    int             bytes_written = wiced_hal_write_nvram( nvram_id, data_len, (uint8_t*)p_data, &result );

    WICED_BT_TRACE("NVRAM ID:%d written :%d bytes result:%d\n", nvram_id, bytes_written, result);
    return (bytes_written);
}

/*
 * Read data from the NVRAM and return in the passed buffer
 */
int a2dp_sink_read_nvram( int nvram_id, void *p_data, int data_len)
{
    uint16_t        read_bytes = 0;
    wiced_result_t  result;

    if (data_len >= sizeof(wiced_bt_device_link_keys_t))
    {
        read_bytes = wiced_hal_read_nvram( nvram_id, sizeof(wiced_bt_device_link_keys_t), p_data, &result );
        WICED_BT_TRACE("NVRAM ID:%d read out of %d bytes:%d result:%d\n", nvram_id, sizeof(wiced_bt_device_link_keys_t), read_bytes, result );
    }
    return (read_bytes);
}









#ifdef AUDIO_MUTE_UNMUTE_ON_INTERRUPT

void a2dp_sink_interrrupt_handler(void *data, uint8_t port_pin )
{
    WICED_BT_TRACE("gpio_interrupt_handler pin: %d\n", port_pin);
#ifdef CYW20706A2
     /* Get the status of interrupt on P# */
    if ( wiced_hal_gpio_get_pin_interrupt_status( WICED_GPIO_BUTTON ) )
    {
        /* Clear the GPIO interrupt */
        wiced_hal_gpio_clear_pin_interrupt_status(WICED_GPIO_BUTTON);
    }
#else
    if ( wiced_hal_gpio_get_pin_interrupt_status( WICED_GPIO_PIN_BUTTON_1 ) )
    {
        /* Clear the GPIO interrupt */
        wiced_hal_gpio_clear_pin_interrupt_status(WICED_GPIO_PIN_BUTTON_1);
    }
#endif
    a2dp_sink_mute_unmute_audio();
}

void a2dp_sink_set_input_interrupt(void)
{
#ifdef CYW20706A2
#ifndef WICED_COEX_ENABLE
    wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_BUTTON, a2dp_sink_interrrupt_handler, NULL );

    /* Configure GPIO PIN# as input, pull down and interrupt on rising edge and output value is set as low */
    wiced_hal_gpio_configure_pin( WICED_GPIO_BUTTON, WICED_GPIO_BUTTON_SETTINGS( GPIO_EN_INT_RISING_EDGE ), WICED_GPIO_BUTTON_DEFAULT_STATE);
#endif // WICED_COEX_ENABLE
#else
    wiced_platform_register_button_callback( WICED_PLATFORM_BUTTON_1, a2dp_sink_interrrupt_handler, NULL, WICED_PLATFORM_BUTTON_RISING_EDGE);
#endif
}

#endif




