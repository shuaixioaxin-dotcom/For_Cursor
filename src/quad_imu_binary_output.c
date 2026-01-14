/**
 * @file quad_imu_binary_output.c
 * @brief Binary output format for combined IMU data
 * 
 * This file provides functions to pack and send IMU data in a compact binary format
 * for efficient transmission and easier parsing by receiving devices.
 * 
 * Binary Packet Format:
 * +--------+--------+--------+--------+------+------+------+------+--------+
 * | Header | Length | SeqNum | IMU    | IMU1 | IMU2 | IMU3 | IMU4 | CRC16  |
 * | 0x5A   | 0xA5   | 2bytes | Valid  | Data | Data | Data | Data | 2bytes |
 * | 1byte  | 1byte  |        | Mask   |      |      |      |      |        |
 * +--------+--------+--------+--------+------+------+------+------+--------+
 * 
 * Each IMU Data Block (if valid):
 * - Timestamp: 4 bytes (uint32_t)
 * - Accelerometer: 12 bytes (3 x float)
 * - Gyroscope: 12 bytes (3 x float)
 * - Magnetometer: 12 bytes (3 x float)
 * - Euler angles: 12 bytes (3 x float)
 * - Quaternion: 16 bytes (4 x float)
 * - Pressure: 4 bytes (float)
 * Total per IMU: 72 bytes
 * 
 * @date 2024-08-07
 * @version 1.0
 */

#include <stdint.h>
#include <string.h>
#include "hipnuc_dec.h"

/* Binary packet header */
#define BINARY_HEADER_1     0x5A
#define BINARY_HEADER_2     0xA5

/* Size of each IMU data block in bytes */
#define IMU_DATA_BLOCK_SIZE 72

/* Maximum binary packet size */
#define MAX_BINARY_PACKET_SIZE (6 + (4 * IMU_DATA_BLOCK_SIZE) + 2)

/* Number of IMUs */
#define NUM_IMU 4

/**
 * @brief Single IMU data structure for binary output
 */
typedef struct {
    uint32_t timestamp;     /* Timestamp in ms */
    float acc[3];           /* Accelerometer (m/s^2) */
    float gyr[3];           /* Gyroscope (deg/s) */
    float mag[3];           /* Magnetometer (uT) */
    float euler[3];         /* Euler angles: roll, pitch, yaw (deg) */
    float quat[4];          /* Quaternion: w, x, y, z */
    float pressure;         /* Pressure (Pa) */
} __attribute__((packed)) imu_data_block_t;

/**
 * @brief Combined IMU packet header
 */
typedef struct {
    uint8_t header1;        /* 0x5A */
    uint8_t header2;        /* 0xA5 */
    uint16_t seq_num;       /* Sequence number */
    uint8_t valid_mask;     /* Bit mask: bit0=IMU1, bit1=IMU2, etc. */
} __attribute__((packed)) imu_packet_header_t;

/* Static buffer for binary packet */
static uint8_t binary_packet_buf[MAX_BINARY_PACKET_SIZE];
static uint16_t packet_seq_num = 0;

/* External reference to IMU channel data (defined in main.c) */
extern void *get_imu_channel_raw(uint8_t idx);
extern uint8_t get_imu_channel_valid(uint8_t idx);

/**
 * @brief Calculate CRC16-CCITT
 * @param data Pointer to data buffer
 * @param length Length of data
 * @return CRC16 value
 */
static uint16_t calc_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    
    while (length--)
    {
        crc ^= (*data++) << 8;
        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
        }
    }
    
    return crc;
}

/**
 * @brief Pack IMU data into binary format
 * @param imu_raw_array Array of pointers to hipnuc_raw_t structures
 * @param valid_flags Array of validity flags for each IMU
 * @param out_buf Output buffer for binary packet
 * @param out_size Pointer to store output packet size
 * @return 0 on success, -1 on error
 */
int pack_imu_binary_packet(hipnuc_raw_t *imu_raw_array[], uint8_t valid_flags[], 
                           uint8_t *out_buf, uint16_t *out_size)
{
    uint16_t offset = 0;
    uint8_t valid_mask = 0;
    
    /* Calculate valid mask */
    for (uint8_t i = 0; i < NUM_IMU; i++)
    {
        if (valid_flags[i])
            valid_mask |= (1 << i);
    }
    
    /* Build header */
    imu_packet_header_t *header = (imu_packet_header_t *)out_buf;
    header->header1 = BINARY_HEADER_1;
    header->header2 = BINARY_HEADER_2;
    header->seq_num = packet_seq_num++;
    header->valid_mask = valid_mask;
    offset = sizeof(imu_packet_header_t);
    
    /* Pack each valid IMU's data */
    for (uint8_t i = 0; i < NUM_IMU; i++)
    {
        if (valid_flags[i] && imu_raw_array[i] != NULL)
        {
            imu_data_block_t *block = (imu_data_block_t *)(out_buf + offset);
            hipnuc_raw_t *raw = imu_raw_array[i];
            
            block->timestamp = raw->hi91.ts;
            
            block->acc[0] = raw->hi91.acc[0];
            block->acc[1] = raw->hi91.acc[1];
            block->acc[2] = raw->hi91.acc[2];
            
            block->gyr[0] = raw->hi91.gyr[0];
            block->gyr[1] = raw->hi91.gyr[1];
            block->gyr[2] = raw->hi91.gyr[2];
            
            block->mag[0] = raw->hi91.mag[0];
            block->mag[1] = raw->hi91.mag[1];
            block->mag[2] = raw->hi91.mag[2];
            
            block->euler[0] = raw->hi91.euler[0];
            block->euler[1] = raw->hi91.euler[1];
            block->euler[2] = raw->hi91.euler[2];
            
            block->quat[0] = raw->hi91.quat[0];
            block->quat[1] = raw->hi91.quat[1];
            block->quat[2] = raw->hi91.quat[2];
            block->quat[3] = raw->hi91.quat[3];
            
            block->pressure = raw->hi91.prs;
            
            offset += sizeof(imu_data_block_t);
        }
    }
    
    /* Calculate and append CRC16 */
    uint16_t crc = calc_crc16(out_buf, offset);
    out_buf[offset++] = (crc >> 8) & 0xFF;  /* CRC high byte */
    out_buf[offset++] = crc & 0xFF;          /* CRC low byte */
    
    *out_size = offset;
    return 0;
}

/**
 * @brief Send binary packet via USART
 * @param usart USART peripheral (e.g., USART1)
 * @param data Data buffer to send
 * @param size Size of data to send
 */
void send_binary_packet(void *usart, const uint8_t *data, uint16_t size)
{
    /* Note: This is a placeholder. The actual implementation depends on 
     * your USART driver. You can use polling, interrupt, or DMA transmission.
     * 
     * Example with STM32 Standard Peripheral Library:
     * 
     * USART_TypeDef *uart = (USART_TypeDef *)usart;
     * for (uint16_t i = 0; i < size; i++)
     * {
     *     while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);
     *     USART_SendData(uart, data[i]);
     * }
     */
    (void)usart;
    (void)data;
    (void)size;
}

/**
 * @brief Get binary packet buffer
 * @return Pointer to static binary packet buffer
 */
uint8_t* get_binary_packet_buffer(void)
{
    return binary_packet_buf;
}

/**
 * @brief Get maximum binary packet size
 * @return Maximum packet size in bytes
 */
uint16_t get_max_binary_packet_size(void)
{
    return MAX_BINARY_PACKET_SIZE;
}
