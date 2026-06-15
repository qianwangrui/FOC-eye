#include "qwr_fdcan_driver.h"
#include <string.h>

FDCAN_HandleTypeDef hfdcan1;
FDCAN_LoopbackStats_t g_fdcan_lb = {0};

static uint8_t dlc_to_len(uint32_t dlc)
{
    switch (dlc) {
    case FDCAN_DLC_BYTES_0:  return 0U;
    case FDCAN_DLC_BYTES_1:  return 1U;
    case FDCAN_DLC_BYTES_2:  return 2U;
    case FDCAN_DLC_BYTES_3:  return 3U;
    case FDCAN_DLC_BYTES_4:  return 4U;
    case FDCAN_DLC_BYTES_5:  return 5U;
    case FDCAN_DLC_BYTES_6:  return 6U;
    case FDCAN_DLC_BYTES_7:  return 7U;
    default:                 return 8U;
    }
}

static uint32_t len_to_dlc(uint8_t len)
{
    static const uint32_t table[9] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3,
        FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7,
        FDCAN_DLC_BYTES_8,
    };
    if (len > 8U) {
        len = 8U;
    }
    return table[len];
}

static void fdcan_config_clock(void)
{
    RCC_PeriphCLKInitTypeDef clk = {0};
    clk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    clk.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&clk) != HAL_OK) {
        while (1) {}
    }
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &gpio);
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    __HAL_RCC_FDCAN_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
}

void FDCAN_Init(void)
{
    fdcan_config_clock();

    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.ClockDivider       = FDCAN_CLOCK_DIV2;
    hfdcan1.Init.FrameFormat        = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode               = FDCAN_MODE_INTERNAL_LOOPBACK;
    hfdcan1.Init.AutoRetransmission = DISABLE;
    hfdcan1.Init.TransmitPause      = DISABLE;
    hfdcan1.Init.ProtocolException  = DISABLE;
    /* PLLQ=170 MHz, /2 => 85 MHz kernel; 85M / (10*17) = 500 kbps classic CAN. */
    hfdcan1.Init.NominalPrescaler     = 10U;
    hfdcan1.Init.NominalSyncJumpWidth = 1U;
    hfdcan1.Init.NominalTimeSeg1      = 14U;
    hfdcan1.Init.NominalTimeSeg2      = 2U;
    hfdcan1.Init.DataPrescaler          = 1U;
    hfdcan1.Init.DataSyncJumpWidth      = 1U;
    hfdcan1.Init.DataTimeSeg1           = 1U;
    hfdcan1.Init.DataTimeSeg2           = 1U;
    hfdcan1.Init.StdFiltersNbr          = 1U;
    hfdcan1.Init.ExtFiltersNbr          = 0U;
    hfdcan1.Init.TxFifoQueueMode        = FDCAN_TX_FIFO_OPERATION;

    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        while (1) {}
    }

    FDCAN_FilterTypeDef filter = {0};
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0U;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x000U;
    filter.FilterID2    = 0x000U;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
        while (1) {}
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE) != HAL_OK) {
        while (1) {}
    }
}

void FDCAN_Start(void)
{
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        while (1) {}
    }
}

uint8_t FDCAN_SendStd(uint16_t std_id, const uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef hdr = {0};
    uint8_t payload[8] = {0};

    if (len > 8U) {
        len = 8U;
    }
    if (data && len > 0U) {
        memcpy(payload, data, len);
    }

    hdr.Identifier              = std_id & 0x7FFU;
    hdr.IdType                  = FDCAN_STANDARD_ID;
    hdr.TxFrameType             = FDCAN_DATA_FRAME;
    hdr.DataLength              = len_to_dlc(len);
    hdr.ErrorStateIndicator     = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch           = FDCAN_BRS_OFF;
    hdr.FDFormat                = FDCAN_CLASSIC_CAN;
    hdr.TxEventFifoControl      = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker           = 0U;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U) {
        return 0U;
    }
    return (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr, payload) == HAL_OK) ? 1U : 0U;
}

uint8_t FDCAN_TryRecvStd(uint32_t *std_id, uint8_t *data, uint8_t *len)
{
    FDCAN_RxHeaderTypeDef hdr = {0};
    uint8_t payload[8] = {0};

    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0U) {
        return 0U;
    }
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &hdr, payload) != HAL_OK) {
        return 0U;
    }

    if (std_id) {
        *std_id = hdr.Identifier;
    }
    if (len) {
        *len = dlc_to_len(hdr.DataLength);
    }
    if (data && len) {
        memcpy(data, payload, *len);
    }
    return 1U;
}

void FDCAN_LoopbackPoll(void)
{
    static uint8_t seq = 0U;
    uint8_t tx[8] = {0xA5U, 0x5AU, 0U, 1U, 2U, 3U, 4U, 5U};
    uint8_t rx[8] = {0};
    uint32_t rx_id = 0U;
    uint8_t rx_len = 0U;
    uint32_t t0;

    tx[2] = seq++;

    if (!FDCAN_SendStd(FDCAN_LB_TEST_ID, tx, sizeof(tx))) {
        g_fdcan_lb.rx_fail_cnt++;
        return;
    }
    g_fdcan_lb.tx_cnt++;

    t0 = HAL_GetTick();
    while ((int32_t)(HAL_GetTick() - t0) < 5) {
        if (FDCAN_TryRecvStd(&rx_id, rx, &rx_len)) {
            g_fdcan_lb.last_rx_id = rx_id;
            g_fdcan_lb.last_rx_len = rx_len;
            memcpy(g_fdcan_lb.last_rx_data, rx, rx_len);

            if (rx_id == FDCAN_LB_TEST_ID && rx_len == sizeof(tx) &&
                memcmp(tx, rx, sizeof(tx)) == 0) {
                g_fdcan_lb.rx_ok_cnt++;
            } else {
                g_fdcan_lb.rx_fail_cnt++;
            }
            return;
        }
    }

    g_fdcan_lb.rx_fail_cnt++;
}
