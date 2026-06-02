<!-- SPDX-License-Identifier: MIT -->

# BLE Stack Flowcharts

This document describes the internal working flow of the stack itself:

- bootstrap and runtime dispatch
- GAP host entry points
- controller radio phases
- connected link scheduling
- L2CAP routing
- ATT and GATT client flow for central
- ATT and GATT server flow for peripheral
- LL control procedures such as feature exchange, data length update, PHY update, and connection update

Primary stack sources used:

- `stack/core/ble_stack.c`
- `stack/core/ble_runtime.c`
- `stack/host/ble_host_internal.h`
- `stack/host/gap/ble_gap.c`
- `stack/controller/ble_controller_state_internal.h`
- `stack/controller/ble_controller_common.c`
- `stack/controller/ble_controller_central.c`
- `stack/controller/ble_controller_peripheral.c`
- `stack/host/l2cap/ble_l2cap.c`
- `stack/host/gatt/ble_gatt_client.c`
- `stack/host/gatt/ble_gatt_server.c`

## Common Bootstrap

Common bootstrap is the same for both roles:

1. `ble_stack_init(role)` validates the role and resets `m_host`, `m_link`, `m_ctrl_rt`, and handler pointers.
2. `controller_load_identity_address()` derives a static random identity address into `m_ctrl_rt.local_addr`.
3. `ble_evt_dispatch_init()` initializes the deferred event queue and enables `SWI1_EGU1_IRQn`.
4. `controller_runtime_init()` initializes `app_timer`, advertising timer, central scan timers, radio hardware, and `TIMER0` for connected events.
5. `ble_gatt_client_reset_connection_state()` initializes the ATT MTU and clears any client-side procedure state.

The shared deferred event path is also common:

1. Internal code posts GAP, GATT, scan report, and characteristic events with `ble_evt_post()`.
2. `ble_evt_post()` pushes the event into the ring buffer and pends `SWI1_EGU1_IRQn`.
3. `SWI1_EGU1_IRQHandler()` drains the queue and dispatches to the registered GAP, GATT server, GATT client, scan-report, or characteristic callback consumer.
4. The same interrupt also starts or stops the peripheral connection-parameter-update timer on `BLE_GAP_EVT_CONNECTED` and `BLE_GAP_EVT_DISCONNECTED`.

## Central Role

### Central Stack Flow

```mermaid
flowchart TD
    A[ble_stack_init CENTRAL] --> B[Reset host, link, controller runtime]
    B --> C[Load static random identity address]
    C --> D[Init deferred event queue and SWI dispatcher]
    D --> E[Init controller runtime<br/>app_timer, scan timers, radio, TIMER0]
    E --> F[ble_gap_scan_init]
    F --> G[ble_gap_set_scan_filter]
    G --> H[ble_gap_start_scanning]
    H --> I[controller_central_start_scanning_internal]
    I --> J[prepare radio common for advertising RX<br/>scanning=true<br/>scan_channel_index=0<br/>start repeated scan timer]

    J --> K[controller_central_scan_timer_handler]
    K --> L[Select adv channel 37, 38, or 39<br/>clear adv_rx_pdu<br/>configure RX radio<br/>scan_radio_phase=WAIT_RX_DISABLED<br/>start scan window timer]

    L --> M{RADIO event while not connected}
    M -->|CRC_OK on ADV| N[controller_central_handle_scan_crc_ok]
    N --> O[Publish scan report through ble_evt_post]
    N --> P{Should we connect now?}
    P -->|Yes, filter or saved candidate matches| Q[Stop scan timer<br/>save connect_target<br/>build CONNECT_REQ<br/>connect_req_tx_pending=true]
    P -->|No, active scan and peer is scannable| R[Build SCAN_REQ<br/>save scan_rsp_target<br/>scan_req_tx_pending=true]
    P -->|No follow-up| S[Wait for DISABLED and return to scan idle]

    M -->|DISABLED in WAIT_RX_DISABLED| T{Pending follow-up?}
    T -->|CONNECT_REQ pending| U[scan_radio_phase=WAIT_CONNECT_TX_DISABLED<br/>TX CONNECT_REQ]
    T -->|SCAN_REQ pending| V[scan_radio_phase=WAIT_SCAN_REQ_TX_DISABLED<br/>TX SCAN_REQ then RX SCAN_RSP]
    T -->|None| W[Stop scan window timer<br/>controller_central_reset_scan_state]

    M -->|DISABLED in WAIT_SCAN_REQ_TX_DISABLED| X[scan_radio_phase=WAIT_SCAN_RSP_RX_DISABLED]
    M -->|CRC_OK on SCAN_RSP| Y[controller_central_handle_scan_rsp_crc_ok<br/>publish scan response report<br/>optionally save connect candidate]
    M -->|DISABLED after SCAN_RSP RX| W

    M -->|DISABLED after CONNECT_REQ TX| Z[controller_central_apply_connect_request]
    Z --> AA[controller_prepare_connected_link role=CENTRAL]
    AA --> AB[connected=true<br/>load access address, crc init, channel map<br/>reset connection runtime<br/>reset GATT server and client connection state<br/>prepare data-channel radio]
    AB --> AC[controller_central_auto_ctrl_start]
    AB --> AD[Post BLE_GAP_EVT_CONNECTED through ble_evt_post]

    AC --> AE[Auto LL control chain<br/>FEATURE_REQ -> LENGTH_REQ -> PHY_REQ -> PHY_UPDATE_IND]
    AD --> AF[SWI1_EGU1_IRQHandler dispatches CONNECTED event]

    AF --> AG[TIMER0 drives first and subsequent connection events]
    AG --> AH[controller_central_start_connection_event<br/>hop data channel<br/>select next TX source:<br/>control response, control, L2CAP, or empty PDU<br/>TX then RX]

    AH --> AI{RADIO event while connected}
    AI -->|CRC_OK| AJ[Update SN and NESN state<br/>track TX ack and new RX packet]
    AI -->|CRC_ERROR| AK[Keep sequence state unchanged]
    AI -->|DISABLED| AL[Finish TX/RX phase<br/>if new packet pending -> controller_process_received_conn_pdu]

    AL --> AM{Received LLID}
    AM -->|Control PDU| AN[ll_control_process]
    AN --> AO[Handle feature exchange, data length update,<br/>PHY update, conn update, terminate]
    AM -->|L2CAP start or continuation| AP[ble_l2cap_process_conn_data_pdu]

    AP --> AQ{L2CAP CID}
    AQ -->|ATT| AR[ble_gatt_client_process_att_pdu]
    AR --> AS[Handle ATT MTU exchange, service discovery,<br/>characteristic discovery, descriptor discovery,<br/>read response, write response,<br/>notification, indication, error response]
    AQ -->|Signaling| AT[Process peer conn-param-update request<br/>possibly call controller_central_initiate_conn_update]

    AO --> AU[Post GAP events through ble_evt_post]
    AS --> AV[Post GATT client events through ble_evt_post<br/>and auto-generate ATT Handle Value Confirmation for indications]
    AT --> AU

    O --> AW[SWI1_EGU1_IRQHandler drains deferred queue]
    AU --> AW
    AV --> AW

    AG --> AX{Disconnect path?}
    AX -->|Supervision timeout| AY[Post GAP supervision timeout event<br/>controller_disconnect_internal]
    AX -->|Terminate indication| AZ[Post GAP terminate event<br/>controller_disconnect_internal]
    AX -->|Explicit or internal disconnect| BA[controller_disconnect_internal]
    AY --> BB[Clear link state, stop timers,<br/>reset GATT connection state,<br/>post DISCONNECTED]
    AZ --> BB
    BA --> BB
```

### Central State Machine

```mermaid
stateDiagram-v2
    [*] --> CentralReady
    CentralReady: Stack initialized for CENTRAL
    CentralReady --> ScanEngine: ble_gap_start_scanning

    state "Scan Engine" as ScanEngine {
        [*] --> ScanIdle
        ScanIdle --> WaitRxDisabled: scan timer arms RX on adv channel
        WaitRxDisabled --> WaitScanReqTxDisabled: active scan needed
        WaitScanReqTxDisabled --> WaitScanRspRxDisabled: SCAN_REQ TX complete
        WaitScanRspRxDisabled --> ScanIdle: SCAN_RSP handled or timeout
        WaitRxDisabled --> WaitConnectTxDisabled: connectable match found
        WaitConnectTxDisabled --> ScanIdle: CONNECT_REQ TX complete
        WaitRxDisabled --> ScanIdle: no follow-up or scan window timeout
    }

    ScanEngine --> ConnectedLink: apply_connect_request

    state "Connected Link" as ConnectedLink {
        [*] --> AutoControl

        state "Auto LL Control Stage" as AutoControl {
            [*] --> WaitFeatures
            WaitFeatures --> WaitDataLength: feature exchange complete
            WaitDataLength --> WaitPhy: data length update complete
            WaitPhy --> AutoDone: PHY update complete or skipped
        }

        state "Central LL Control Procedure" as CentralCtrlProc {
            [*] --> CtrlIdle
            CtrlIdle --> WaitRsp: FEATURE_REQ, LENGTH_REQ, or PHY_REQ queued
            WaitRsp --> CtrlIdle: FEATURE_RSP or LENGTH_RSP completes
            WaitRsp --> WaitInstant: PHY_RSP schedules PHY_UPDATE_IND
            CtrlIdle --> WaitInstant: local CONN_UPDATE_IND queued
            WaitInstant --> CtrlIdle: instant reached in TIMER0 path
        }

        state "Connection Event Radio" as ConnRadio {
            [*] --> ConnWaitTxDisabled
            ConnWaitTxDisabled --> ConnWaitRxDisabled: TX complete
            ConnWaitRxDisabled --> ConnIdle: RX complete
            ConnIdle --> ConnWaitTxDisabled: next connection anchor
        }

        state "GATT Client Procedure" as GattClient {
            [*] --> GattIdle
            GattIdle --> MtuExchange: ATT MTU request queued
            MtuExchange --> DiscoverServiceByUuid: MTU exchange complete
            DiscoverServiceByUuid --> DiscoverCharacteristics: service procedure complete
            DiscoverCharacteristics --> DiscoverDescriptors: characteristic procedure complete
            DiscoverDescriptors --> WriteCccd: descriptor procedure complete
            WriteCccd --> Subscribed: CCCD write response complete
            Subscribed --> Subscribed: reads, writes, notifications, indications
        }
    }

    ConnectedLink --> CentralDisconnected: disconnect or supervision timeout
    CentralDisconnected --> ScanEngine: ble_gap_start_scanning called again
```

### Central Internal Notes

- Scan-phase state is carried by `m_ctrl_rt.central.scan_radio_phase`.
- Connected-phase radio state is carried by `m_ctrl_rt.conn.conn_radio_phase`.
- Automatic post-connect LL negotiation is central-only and begins inside `controller_prepare_connected_link()` through `controller_central_auto_ctrl_start()`.
- Connection events are anchored by `TIMER0_IRQHandler()`, which also:
  - checks supervision timeout
  - applies pending channel-map updates
  - applies pending connection-parameter updates at the instant
  - applies pending PHY updates at the instant
- ATT traffic arrives over L2CAP CID `BLE_L2CAP_CID_ATT` and is decoded by `ble_gatt_client_process_att_pdu()`.
- L2CAP signaling traffic arrives over CID `BLE_L2CAP_CID_SIGNALING` and is handled by `ble_l2cap_process_signaling_pdu()`. In central role it is where a peripheral connection-parameter update request is accepted or rejected.

## Peripheral Role

### Peripheral Stack Flow

```mermaid
flowchart TD
    A[ble_stack_init PERIPHERAL] --> B[Reset host, link, controller runtime]
    B --> C[Load static random identity address]
    C --> D[Init deferred event queue and SWI dispatcher]
    D --> E[Init controller runtime<br/>app_timer, adv timer, radio, TIMER0]
    E --> F[ble_gap_adv_init<br/>copy adv and scan-response config<br/>copy service UUID lists]
    F --> G[ble_gatt_server_init]
    G --> H[Build fixed and custom GATT attributes<br/>reset ATT MTU, CCCD state, indication-pending state]
    H --> I[ble_gap_start_advertising]
    I --> J[controller_peripheral_start_advertising_internal]
    J --> K[prepare radio common for advertising TX<br/>enable ADV IRQ mask<br/>start repeated advertising timer]

    K --> L[controller_peripheral_adv_timer_handler]
    L --> M[Build ADV PDU]
    M --> N[Build SCAN_RSP PDU]
    N --> O[Advertising event across channels 37, 38, 39]
    O --> P[Per channel:<br/>clear adv_rx_pdu<br/>reset adv state<br/>radio TX then RX<br/>adv_radio_phase=WAIT_ADV_TX_DISABLED]

    P --> Q{RADIO event while not connected}
    Q -->|DISABLED in WAIT_ADV_TX_DISABLED| R[adv_radio_phase=WAIT_RX_DISABLED]
    Q -->|CRC_OK in WAIT_RX_DISABLED| S[controller_peripheral_handle_adv_crc_ok]
    S --> T{Valid request for our address?}
    T -->|Valid SCAN_REQ and role is scannable| U[scan_rsp_tx_pending=true]
    T -->|Valid CONNECT_REQ and role is connectable| V[connect_req_pending=true]
    T -->|No valid request| W[Wait for DISABLED]

    Q -->|DISABLED in WAIT_RX_DISABLED| X{Pending follow-up?}
    X -->|SCAN_RSP pending| Y[adv_radio_phase=WAIT_SCAN_RSP_TX_DISABLED<br/>TX SCAN_RSP]
    X -->|CONNECT_REQ pending| Z[reset adv state<br/>radio_disable<br/>controller_peripheral_apply_connect_request]
    X -->|None| AA[reset advertising radio state]

    Q -->|DISABLED in WAIT_SCAN_RSP_TX_DISABLED| AA

    Z --> AB[Stop advertising timer]
    AB --> AC[controller_prepare_connected_link role=PERIPHERAL]
    AC --> AD[connected=true<br/>load access address, crc init, channel map<br/>reset connection runtime<br/>reset GATT server and client connection state<br/>prepare data-channel radio]
    AD --> AE[Start connected-event timer]
    AE --> AF[Post BLE_GAP_EVT_CONNECTED through ble_evt_post]
    AF --> AG[SWI1_EGU1_IRQHandler dispatches CONNECTED<br/>and starts delayed conn-param-update timer]

    AG --> AH[TIMER0 drives first and subsequent connection events]
    AH --> AI[controller_peripheral_start_connection_event<br/>hop data channel<br/>open RX first<br/>enable BCMATCH for data header<br/>pre-stage turnaround to TX]

    AI --> AJ{RADIO event while connected}
    AJ -->|BCMATCH| AK[Pre-stage response from header state<br/>predict TX ack and new RX packet]
    AJ -->|CRC_OK| AL[Update sequence state<br/>schedule next compare<br/>stage response PDU]
    AJ -->|CRC_ERROR| AM[Keep sequence state<br/>stage retransmission-friendly response]
    AJ -->|DISABLED| AN[Finish RX/TX phase<br/>if new packet pending -> controller_process_received_conn_pdu]

    AN --> AO{Received LLID}
    AO -->|Control PDU| AP[ll_control_process]
    AP --> AQ[Handle feature exchange response,<br/>data length response,<br/>incoming conn update indication,<br/>incoming PHY update indication,<br/>terminate indication]
    AO -->|L2CAP start or continuation| AR[ble_l2cap_process_conn_data_pdu]

    AR --> AS{L2CAP CID}
    AS -->|ATT| AT[ble_gatt_server_process_att_pdu]
    AT --> AU[Handle ATT MTU exchange,<br/>service discovery queries,<br/>characteristic and descriptor queries,<br/>read requests,<br/>write requests and commands,<br/>handle value confirmations]
    AS -->|Signaling| AV[No peripheral-side special signaling handler for responses<br/>requests are mainly originated outbound]

    AG --> AW[After delay, ble_gap_request_conn_params_update]
    AW --> AX[ble_l2cap_queue_conn_param_update_req]
    AX --> AY[Queue L2CAP signaling PDU for next connection event]
    AY --> AZ[Peer central may reply and later send LL CONN_UPDATE_IND]
    AZ --> BA[Common TIMER0 path applies pending connection update at instant]

    BA --> BB[Post BLE_GAP_EVT_CONN_UPDATE_IND through ble_evt_post]
    AQ --> BC[Post GAP events through ble_evt_post]

    BD[ble_gatt_server_notify_characteristic or ble_gatt_server_indicate_characteristic] --> BE[Build ATT Handle Value Notification or Indication]
    BE --> BF[Queue ATT payload through controller_queue_l2cap_payload]
    BF --> BG[Sent during next connection event]
    BG --> BH{If indication?}
    BH -->|Yes| BI[m_indication_pending=true until Handle Value Confirmation]
    BH -->|No| BJ[Return immediately after queue]

    AU --> BK[Characteristic write or CCCD change can post deferred characteristic events]
    BK --> BL[SWI1_EGU1_IRQHandler dispatches characteristic callbacks]
    BB --> BM[SWI1_EGU1_IRQHandler dispatches GAP update event]
    BC --> BM

    AH --> BN{Disconnect path?}
    BN -->|Supervision timeout| BO[Post GAP supervision timeout event<br/>controller_disconnect_internal]
    BN -->|Terminate indication| BP[Post GAP terminate event<br/>controller_disconnect_internal]
    BN -->|Explicit or internal disconnect| BQ[controller_disconnect_internal]
    BO --> BR[Clear link state, stop timers,<br/>reset GATT connection state,<br/>post DISCONNECTED]
    BP --> BR
    BQ --> BR
```

### Peripheral State Machine

```mermaid
stateDiagram-v2
    [*] --> PeripheralReady
    PeripheralReady: Stack initialized for PERIPHERAL
    PeripheralReady --> Advertising: ble_gap_start_advertising

    state "Advertising Engine" as Advertising {
        [*] --> AdvIdle
        AdvIdle --> WaitAdvTxDisabled: advertising timer starts TX
        WaitAdvTxDisabled --> WaitRxDisabled: ADV TX complete
        WaitRxDisabled --> WaitScanRspTxDisabled: valid SCAN_REQ detected
        WaitScanRspTxDisabled --> AdvIdle: SCAN_RSP TX complete
        WaitRxDisabled --> AdvIdle: no valid follow-up request
        WaitRxDisabled --> AdvIdle: RX window closed manually
    }

    Advertising --> PeripheralConnected: valid CONNECT_REQ applied

    state "Connected Link" as PeripheralConnected {
        [*] --> ServerReady

        state "Connection Event Radio" as PeriphConnRadio {
            [*] --> ConnWaitRxDisabled
            ConnWaitRxDisabled --> ConnWaitTxDisabled: RX complete, response staged
            ConnWaitTxDisabled --> ConnIdle: TX complete
            ConnIdle --> ConnWaitRxDisabled: next connection anchor
        }

        state "Server ATT or GATT" as ServerReady {
            [*] --> SrvIdle
            SrvIdle --> MtuExchange: ATT MTU request
            MtuExchange --> SrvIdle: MTU response queued
            SrvIdle --> DiscoveryResponse: service, characteristic, descriptor, or read request
            DiscoveryResponse --> SrvIdle: response queued
            SrvIdle --> WriteHandling: write request or write command
            WriteHandling --> SrvIdle: value stored and events emitted
            SrvIdle --> NotifyQueued: notification queued
            NotifyQueued --> SrvIdle: next connection event transmits it
            SrvIdle --> IndicationPending: indication queued
            IndicationPending --> SrvIdle: Handle Value Confirmation received
        }

        state "Conn Param Update Request Path" as ConnParamPath {
            [*] --> TimerStopped
            TimerStopped --> TimerArmed: deferred CONNECTED event handled
            TimerArmed --> ReqQueued: 6-second timer expires and queues L2CAP signaling request
            ReqQueued --> WaitLlUpdateInstant: peer later sends LL CONN_UPDATE_IND
            WaitLlUpdateInstant --> TimerArmed: TIMER0 instant applies new parameters
        }
    }

    PeripheralConnected --> PeripheralDisconnected: disconnect or supervision timeout
    PeripheralDisconnected --> Advertising: ble_gap_start_advertising called again
```

### Peripheral Internal Notes

- Advertising-phase state is carried by `m_ctrl_rt.peripheral.adv_radio_phase`.
- Advertising and scan-response configuration is stored in separate
  `ble_host_adv_data_t` blocks. Name, TX power, and service UUID list metadata
  are copied into host-owned storage during `ble_gap_adv_init()`.
- Service data and manufacturer-specific data metadata are copied, but their
  payload pointers remain application-owned so the application can update those
  buffers between advertising events.
- Complete service UUID lists are emitted as incomplete lists if only part of
  the configured list fits in the selected legacy advertising packet.
- Connected-phase radio state is shared with central through `m_ctrl_rt.conn.conn_radio_phase`, but the peripheral runs RX first and relies on `BCMATCH` to pre-stage the response.
- Outbound server notifications and indications are not sent immediately on the API call. They are queued as ATT payloads and transmitted during the next connected connection event.
- Indications use `m_indication_pending` as a gate. Another indication cannot be queued until `BLE_ATT_OP_HANDLE_VALUE_CONFIRMATION` is received.
- Peripheral connection-parameter update is a two-stage path:
  - the peripheral queues an L2CAP signaling request
  - the actual effective parameter change still happens later through LL `CONN_UPDATE_IND` and the shared instant-based connection update logic in `TIMER0_IRQHandler()`

## Role Split Summary

- Central-specific internals:
  - scan engine and connect decision
  - automatic LL feature, DLE, and PHY negotiation
  - GATT client ATT flow
  - handling incoming peripheral connection-parameter-update requests

- Peripheral-specific internals:
  - advertising engine and request targeting
  - scan-response and connect-request handling
  - GATT server ATT flow
  - delayed outbound L2CAP connection-parameter-update request generation

- Shared internals:
  - identity address load
  - deferred event queue
  - radio common configuration
  - connected link scheduling
  - supervision timeout handling
  - LL control processing
  - L2CAP routing
