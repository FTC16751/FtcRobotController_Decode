package org.firstinspires.ftc.teamcode.utilities.Common.prismled;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Minimal I2C wrapper for goBILDA Prism RGB LED Driver.
 * Primary use: load artboards (saved scenes) during TeleOp/Auto.
 */
public class PrismI2c {

    public static final int DEFAULT_7BIT_ADDR = 0x38;

    // Prism register map (from user guide)
    private static final int REG_CONTROL            = 0x06;
    private static final int REG_ARTBOARD_LOAD_SAVE = 0x07;
    private static final int REG_LAYER0             = 0x08; // Layer 0 base register

    /**
     * If artboard switching doesn't work, flip this.
     * The artboard register takes a 32-bit bitmask; endianness can be the only gotcha.
     */
    public static boolean LITTLE_ENDIAN = true;

    private final I2cDeviceSynch device;

    public PrismI2c(I2cDeviceSynch device) {
        this.device = device;
        this.device.setI2cAddress(I2cAddr.create7bit(DEFAULT_7BIT_ADDR));
        this.device.engage();
    }

    public static PrismI2c fromHardwareMap(HardwareMap hw, String configName) {
        return new PrismI2c(hw.get(I2cDeviceSynch.class, configName));
    }

    /** Load and display an artboard slot (0..7). */
    public void loadArtboard(int slot) {
        if (slot < 0 || slot > 7) return;
        int mask = 1 << (8 + slot);
        writeU32(REG_ARTBOARD_LOAD_SAVE, mask);
    }

    /** Save current working artboard to slot (0..7). */
    public void saveArtboard(int slot) {
        if (slot < 0 || slot > 7) return;
        int mask = 1 << slot;
        writeU32(REG_ARTBOARD_LOAD_SAVE, mask);
    }

    /** Optional: clear current animations (useful for debugging). */
    public void clearCurrentAnimation() {
        // Control register bit 2: Clear Current Animation (per Prism guide)
        device.write8(REG_CONTROL, (byte) (1 << 2));
    }

    /**
     * Optional: set Layer 0 selected animation.
     * Layer 0 register is 0x08. Sub-register 0x00 is "Selected Animation".
     */
    public void setLayer0SelectedAnimation(byte animationId) {
        device.write(REG_LAYER0, new byte[]{0x00, animationId});
    }

    /** Write a 32-bit value as 4 bytes to a register. */
    public void writeU32(int reg, int value) {
        ByteBuffer bb = ByteBuffer.allocate(4);
        bb.order(LITTLE_ENDIAN ? ByteOrder.LITTLE_ENDIAN : ByteOrder.BIG_ENDIAN);
        bb.putInt(value);
        device.write(reg, bb.array());
    }
}

