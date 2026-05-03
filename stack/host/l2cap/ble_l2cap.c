/**
 * @file ble_l2cap.c
 * @author Surya Poudel
 * @brief Internal L2CAP signaling helpers for the BLE stack
 * @version 0.1
 * @date 2026-05-01
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ble_l2cap_internal.h"

#include "ble_gatt_client_internal.h"
#include "ble_gatt_server_internal.h"
#include "ble_runtime_internal.h"

static uint16_t ble_l2cap_process_signaling(const uint8_t *p_sig,
                                            uint16_t sig_len,
                                            uint8_t *p_rsp,
                                            uint16_t rsp_max_len);

bool ble_l2cap_queue_conn_param_update_req(const ble_gap_conn_params_t *p_params)
{
    uint8_t sig_pdu[BLE_L2CAP_SIG_HDR_LEN + 8U];

    if (p_params == NULL)
    {
        return false;
    }

    sig_pdu[0] = BLE_L2CAP_SIG_CONN_PARAM_UPDATE_REQ;
    m_host.common.next_l2cap_sig_identifier++;
    if (m_host.common.next_l2cap_sig_identifier == 0U)
    {
        m_host.common.next_l2cap_sig_identifier = 1U;
    }
    sig_pdu[1] = m_host.common.next_l2cap_sig_identifier;
    u16_encode(8U, &sig_pdu[2]);
    u16_encode(p_params->min_conn_interval_1p25ms, &sig_pdu[4]);
    u16_encode(p_params->max_conn_interval_1p25ms, &sig_pdu[6]);
    u16_encode(p_params->slave_latency, &sig_pdu[8]);
    u16_encode(p_params->supervision_timeout_10ms, &sig_pdu[10]);

    return controller_queue_l2cap_payload(BLE_L2CAP_CID_SIGNALING, sig_pdu, sizeof(sig_pdu));
}

void ble_l2cap_process_conn_data_pdu(const uint8_t *p_payload, uint8_t pdu_len)
{
    uint16_t l2cap_len;
    uint16_t l2cap_cid;
    uint16_t rsp_cid = 0U;
    uint16_t rsp_len = 0U;
    uint8_t rsp[BLE_ATT_MAX_MTU];

    if ((p_payload == NULL) || (pdu_len < BLE_L2CAP_HDR_LEN))
    {
        return;
    }

    l2cap_len = u16_decode(&p_payload[0]);
    l2cap_cid = u16_decode(&p_payload[2]);

    if ((uint16_t)pdu_len < (uint16_t)(l2cap_len + BLE_L2CAP_HDR_LEN))
    {
        return;
    }

    if (l2cap_cid == BLE_L2CAP_CID_ATT)
    {
        rsp_cid = BLE_L2CAP_CID_ATT;
        rsp_len = (m_link.role == BLE_GAP_ROLE_CENTRAL)
                      ? ble_gatt_client_process_pdu(&p_payload[4], l2cap_len, rsp, sizeof(rsp))
                      : ble_gatt_server_process_request(&p_payload[4], l2cap_len, rsp, sizeof(rsp));
    }
    else if (l2cap_cid == BLE_L2CAP_CID_SIGNALING)
    {
        rsp_cid = BLE_L2CAP_CID_SIGNALING;
        rsp_len = ble_l2cap_process_signaling(&p_payload[4], l2cap_len, rsp, sizeof(rsp));
    }

    if (rsp_len > 0U)
    {
        (void)controller_queue_l2cap_payload(rsp_cid, rsp, rsp_len);
    }
}

static uint16_t ble_l2cap_process_signaling(const uint8_t *p_sig,
                                            uint16_t sig_len,
                                            uint8_t *p_rsp,
                                            uint16_t rsp_max_len)
{
    uint8_t code;
    uint8_t identifier;
    uint16_t length;

    if ((p_sig == NULL) || (sig_len < BLE_L2CAP_SIG_HDR_LEN) || (p_rsp == NULL) || (rsp_max_len < 6U))
    {
        return 0U;
    }

    code = p_sig[0];
    identifier = p_sig[1];
    length = u16_decode(&p_sig[2]);
    if ((uint16_t)(length + BLE_L2CAP_SIG_HDR_LEN) > sig_len)
    {
        return 0U;
    }

    if ((code == BLE_L2CAP_SIG_CONN_PARAM_UPDATE_REQ) &&
        (m_link.role == BLE_GAP_ROLE_CENTRAL) &&
        (length >= 8U))
    {
        ble_gap_conn_params_t requested_params = {
            .min_conn_interval_1p25ms = u16_decode(&p_sig[4]),
            .max_conn_interval_1p25ms = u16_decode(&p_sig[6]),
            .slave_latency = u16_decode(&p_sig[8]),
            .supervision_timeout_10ms = u16_decode(&p_sig[10]),
        };
        bool accept = controller_central_initiate_conn_update(&requested_params);

        p_rsp[0] = BLE_L2CAP_SIG_CONN_PARAM_UPDATE_RSP;
        p_rsp[1] = identifier;
        u16_encode(2U, &p_rsp[2]);
        u16_encode(accept ? BLE_L2CAP_SIG_CONN_PARAM_ACCEPTED : BLE_L2CAP_SIG_CONN_PARAM_REJECTED,
                   &p_rsp[4]);
        return 6U;
    }

    return 0U;
}
