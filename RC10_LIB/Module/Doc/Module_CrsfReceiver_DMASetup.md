CRSF Receiver: UART7/DMA & D-Cache troubleshooting

Steps to check and configure CubeMX and linker for safe DMA with D-Cache (STM32H7):

1. UART & DMA settings in CubeMX
- In "USART7" (UART7) peripheral settings:
  - Baud rate: 420000 (CRSF standard)
  - Word length: 8 bits
  - Parity: None
  - Stop bits: 1
  - Hardware flow control: None

- In "DMA" configuration for UART7 RX:
  - Request: DMA_REQUEST_UART7_RX
  - Mode: Circular (DMA_CIRCULAR)
  - Direction: Peripheral-to-memory
  - Peripheral increment: Disabled
  - Memory increment: Enabled
  - Peripheral data alignment: Byte
  - Memory data alignment: Byte
  - FIFO mode: Disable
  - Priority: Low/Medium (adjust as needed)

- NVIC: Ensure the DMA stream IRQ and the UART IRQ are enabled; set reasonable priorities so UART/DMA IRQs are not preempted by high-frequency interrupts that could starve data handling.

2. Ensure HAL usage is "ReceiveToIdle DMA" if you rely on HAL_UARTEx_ReceiveToIdle_DMA. In generated code, you will see a reference such as:
- HAL_UARTEx_ReceiveToIdle_DMA(&huart7, rx_buffer, RX_SIZE);
- HAL_UARTEx_RxEventCallback() is then used to receive the actual number of bytes in each event.

3. Memory placement for DMA buffers
- If CPU D-Cache is enabled (Cortex-M7), DMA buffers must either be:
  - Placed in non-cacheable D2/AXI SRAM (recommended), or
  - Kept in cacheable memory but carefully synchronized: SCB_CleanDCache_by_Addr/Invalidate calls on the exact aligned range before/after DMA accesses.

- To place buffer in D2 SRAM from CubeMX/LD script:
  - Add a custom section to the buffer variable (in code):
    uint8_t rx_buffer_[256] __attribute__((section(".dma_buffer"), aligned(32)));
  - Modify the linker script to locate .dma_buffer into D2 SRAM region (e.g. RAM_D2, SRAMx); names differ per linker script. For MDK/Keil/ARMCC and GCC, insert a section mapping with ORIGIN/LENGTH matching D2.

  Example linker snippets for GCC (adjust ORIGIN/LENGTH to match your board):

    MEMORY
    {
      FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 2M
      RAM_D1 (xrw) : ORIGIN = 0x24000000, LENGTH = 128K
      RAM_D2 (xrw) : ORIGIN = 0x30000000, LENGTH = 256K
    }

    .dma_buffer (NOLOAD) :
    {
      . = ALIGN(4);
      *(.dma_buffer .dma_buffer.*)
      . = ALIGN(4);
    } > RAM_D2

  Then define the macro in your compiler's preprocessor defines or in a header before including Module_CrsfReceiver.h:

    #define CRSF_DMA_SECTION_NAME ".dma_buffer"


4. Quick testing steps
- Temporarily disable D-Cache at runtime via SCB_DisableDCache() (try in startup) to see if the problem disappears -> if yes, it's likely a caching issue.
- If disabling D-Cache resolves the issue, re-enable D-Cache and either:
  - Move DMA buffers to D2 non-cache memory, or
  - Add SCB_InvalidateDCache_by_Addr on RX DMA buffer before reading it and SCB_CleanDCache_by_Addr on TX buffer before transmit (line-aligned to 32 bytes).

5. Verify generated usart.c
- Confirm code sets hdma_uart7_rx.Init.Mode = DMA_CIRCULAR (c.f. usart.c in the project). Also confirm HAL links the dma handle via __HAL_LINKDMA.

6. Additional helpful checks
- Confirm UART7 RX DMA size and buffer boundaries. If HAL provides len in callback, only read len bytes, not the full RX buffer length.
- Ensure the ReceiveToIdle callback and copies (into ring buffer) invalidate the proper range of cache for the length you will process.

If you want, I can generate a minimal patch to:
- Put `rx_buffer_` in a `.dma_buffer` section and add sample linker script snippets for GCC/MDK.
- Add a small runtime API to toggle D-Cache for testing (already implemented in code patch).

Notes:
- Do not disable/enabling cache during critical real-time sections in production; use only for debugging.
- Use 32-byte alignment for SCB Clean/Invalidate ranges.

Quick runtime test to try in `main()`:

  // disable D-Cache to check if cache is root cause
  radio.setDisableDCacheForTest(true);
  // run your test scenario; if problem disappears, it's a D-Cache issue
  radio.setDisableDCacheForTest(false); // re-enable later
