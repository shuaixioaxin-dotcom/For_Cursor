/**
 * @brief Process IMU data from all channels
 */
static void process_data(void)
{
    static uint16_t wait_timer = 0;
    uint8_t any_new_decoded = 0;

    // 1. Process received data from each channel
    for (int i = 0; i < IMU_COUNT; i++)
    {
        if (new_data_flag[i])
        {
            uint16_t current_len = uart_rx_index[i];
            
            // Mark processed for this interrupt batch
            new_data_flag[i] = 0; 
            
            // Processing loop
            for (uint16_t k = 0; k < current_len; k++)
            {
                if (hipnuc_input(&hipnuc_raw[i], uart_rx_buf[i][k]))
                {
                    /* Packet decoded successfully */
                    packet_ready[i] = 1;
                    any_new_decoded = 1;
                }
            }
            
            // Reset buffer index safely
            IRQn_Type irq_n;
            switch(i) {
                case 0: irq_n = USART2_IRQn; break;
                case 1: irq_n = USART3_IRQn; break;
                case 2: irq_n = UART4_IRQn; break;
                case 3: irq_n = UART5_IRQn; break;
                default: irq_n = USART2_IRQn; break;
            }
            
            NVIC_DisableIRQ(irq_n);
            uart_rx_index[i] = 0;
            NVIC_EnableIRQ(irq_n);
        }
    }

    // 2. Logic for dynamic batch printing
    // If any new packet arrived and we are not already waiting, start the timer.
    if (any_new_decoded && wait_timer == 0)
    {
        wait_timer = 10; // Start 10ms window (assuming ~1ms loop with delay_ms(1))
    }

    // If timer is active, count down
    if (wait_timer > 0)
    {
        wait_timer--;
        
        // Check states
        uint8_t all_ready = 1;
        uint8_t any_ready = 0;
        
        for (int i = 0; i < IMU_COUNT; i++)
        {
            if (!packet_ready[i]) {
                all_ready = 0;
            } else {
                any_ready = 1;
            }
        }

        // Trigger print if:
        // A) All 4 IMUs have data (Optimization to print immediately)
        // B) Timeout expired (Print whatever we have, e.g. 2 or 3 IMUs)
        if (all_ready || wait_timer == 0)
        {
            if (any_ready)
            {
                // Print combined data block
                printf("--- Combined IMU Data ---\r\n");
                for (int i = 0; i < IMU_COUNT; i++)
                {
                    if (packet_ready[i])
                    {
                        // Format packet for this IMU
                        hipnuc_dump_packet(&hipnuc_raw[i], log_buf, sizeof(log_buf));
                        printf("[IMU %d] len:%d: %s\r\n", i + 1, hipnuc_raw[i].len, log_buf);
                        
                        // Clear ready flag
                        packet_ready[i] = 0;
                    }
                }
                printf("\r\n"); // Extra newline for separation
            }
            
            // Stop/Reset timer
            wait_timer = 0;
        }
    }
}
