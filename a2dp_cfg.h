
#define A2DP_SINK_NVRAM_ID                      WICED_NVRAM_VSID_START
#ifndef CYW43012C0
#define AUDIO_MUTE_UNMUTE_ON_INTERRUPT    // if defined, the app will mute/unmute audio while streaming on button press
#endif

extern uint8_t pincode[4];

extern const wiced_bt_cfg_settings_t    wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t    wiced_bt_cfg_buf_pools[];
extern const wiced_bt_audio_config_buffer_t a2dp_sink_audio_buf_config;

void a2dp_sink_write_eir( void );
int a2dp_sink_write_nvram( int nvram_id, int data_len, void *p_data);
int a2dp_sink_read_nvram( int nvram_id, void *p_data, int data_len);
void a2dp_sink_set_input_interrupt(void);
