/*
 * This file has been automatically generated by the WICED 20719-B1 Designer.
 * SDP database for Single Mode BR/EDR or Dual Mode device.
 *
 */

// Zenitron_BT_AMP_sdp_db.c

#include "wiced_bt_uuid.h"
#include "wiced_bt_sdp.h"
#include "Zenitron_BT_AMP_sdp_db.h"

const uint8_t sdp_database[] = // Define SDP database
{
    SDP_ATTR_SEQUENCE_1(HANDLE_ATTR_SEQ),

    // SDP Record for A2DP Sink
    SDP_ATTR_SEQUENCE_1(77),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVDT_SINK),
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_AUDIO_SINK),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVDTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVDTP),
                SDP_ATTR_VALUE_UINT2(0x103),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_ADV_AUDIO_DISTRIBUTION),
                SDP_ATTR_VALUE_UINT2(0x103),
        SDP_ATTR_SERVICE_NAME(12),
            'Z', 'e', 'n', 'i', 't', 'r', 'o', 'n', ' ', 'I', 'o', 'T',
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, 0x000B),


#ifdef USE_AVRCP_CTRLR
    // SDP Record for AVRC Target
    SDP_ATTR_SEQUENCE_1(56),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVRC_TARGET),
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),
            SDP_ATTR_SEQUENCE_1(3),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REM_CTRL_TARGET),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVCTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVCTP),
                SDP_ATTR_VALUE_UINT2(0x100),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_VALUE_UINT2(AVRC_REV_1_0),
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, AVRC_SUPF_TG_CAT1),

    // SDP Record for AVRC Controller
    SDP_ATTR_SEQUENCE_1(59),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVRC_CONTROLLER),
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REM_CTRL_CONTROL),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVCTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVCTP),
                SDP_ATTR_VALUE_UINT2(0x104),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_VALUE_UINT2(AVRC_REV_1_5),
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, AVRC_SUPF_CT_CAT2),
#endif
};

// Length of the SDP database
const uint16_t sdp_database_len = sizeof(sdp_database);
