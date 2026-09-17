#include "wm8960.h"

#define WM8960_ADDR 0x34  //7-bit addr 0x1A, pre-shifted for HAL (write op)

static HAL_StatusTypeDef wm8960_write(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t data)
{
    uint8_t addr_byte = (reg << 1) | ((data >> 8) & 0x01);
    uint8_t data_byte = data & 0xFF;
    return HAL_I2C_Mem_Write(hi2c, WM8960_ADDR, addr_byte, I2C_MEMADD_SIZE_8BIT, &data_byte, 1, HAL_MAX_DELAY);
}

bool WM8960_IsReady(I2C_HandleTypeDef *hi2c)
{
    return HAL_I2C_IsDeviceReady(hi2c, WM8960_ADDR, 5, 100) == HAL_OK;
}

HAL_StatusTypeDef WM8960_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret = HAL_OK;

    //Software reset — must be first
    ret |= wm8960_write(hi2c, 0x0F, 0x001);
    HAL_Delay(10);

    //LOUT1/ROUT1 volume (headphone out), write twice to latch (VU bit)
    ret |= wm8960_write(hi2c, 0x02, 0x1FF);
    ret |= wm8960_write(hi2c, 0x02, 0x1FF);
    ret |= wm8960_write(hi2c, 0x03, 0x1FF);
    ret |= wm8960_write(hi2c, 0x03, 0x1FF);

    //PLL N — prescale /2, fractional mode, N=8 (targets SYSCLK=12.288MHz from 24MHz MCLK)
    ret |= wm8960_write(hi2c, 0x34, 0x038);
    // PLL K (3 bytes)
    ret |= wm8960_write(hi2c, 0x35, 0x31);
    ret |= wm8960_write(hi2c, 0x36, 0x26);
    ret |= wm8960_write(hi2c, 0x37, 0xE8);

    //Clocking(1) — SYSCLKDIV=/2, CLKSEL=PLL output
    ret |= wm8960_write(hi2c, 0x04, 0x005);

    //Audio Interface — FORMAT=I2S, WL=16-bit, MS=0(slave)
    ret |= wm8960_write(hi2c, 0x07, 0x002);

    // Use DACLRC as the frame clock for the ADC.
    // The Waveshare board exposes DACLRC/I2S_LRCLK,
    // but does not expose ADCLRC/GPIO1.
    ret |= wm8960_write(hi2c, 0x09, 0x040);

    // DAC control — unmute digital DAC
    ret |= wm8960_write(hi2c, 0x05, 0x000);

    //DAC L/R volume, 0dB, write twice to latch
    ret |= wm8960_write(hi2c, 0x0A, 0x1FF);
    ret |= wm8960_write(hi2c, 0x0A, 0x1FF);
    ret |= wm8960_write(hi2c, 0x0B, 0x1FF);
    ret |= wm8960_write(hi2c, 0x0B, 0x1FF);

    //ADC L/R volume, 0dB, write twice
    ret |= wm8960_write(hi2c, 0x15, 0x1C3);
    ret |= wm8960_write(hi2c, 0x15, 0x1C3);
    ret |= wm8960_write(hi2c, 0x16, 0x1C3);
    ret |= wm8960_write(hi2c, 0x16, 0x1C3);

    //INPUT PGA GAIN: Left  = 0 dB, Right = 0 dB
    ret |= wm8960_write(hi2c, 0x00, 0x117);
    ret |= wm8960_write(hi2c, 0x01, 0x117);

    //Power Mgmt 1 — VMID, VREF, AIN, ADC on
    ret |= wm8960_write(hi2c, 0x19, 0x0FC);

    //Power Mgmt 2 — DAC, LOUT1/ROUT1, SPK, PLL enable
    ret |= wm8960_write(hi2c, 0x1A, 0x1FB);

    //Input boost mixers (left/right) — left at defaults for now
    ret |= wm8960_write(hi2c, 0x20, 0x100);
    ret |= wm8960_write(hi2c, 0x21, 0x100);

    // DAC → left/right output mixers
    ret |= wm8960_write(hi2c, 0x22, 0x100);
    ret |= wm8960_write(hi2c, 0x25, 0x100);

    //Output mixer enable (LOMIX/ROMIX)
    ret |= wm8960_write(hi2c, 0x2F, 0x03C);

    // Speaker volume: 0 dB, unmuted
    ret |= wm8960_write(hi2c, 0x28, 0x179);
    ret |= wm8960_write(hi2c, 0x29, 0x179);

    // Enable Class-D left + right, preserving reserved/default bits
    ret |= wm8960_write(hi2c, 0x31, 0x0F7);

    return ret;
}
