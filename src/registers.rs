#![allow(dead_code)]
#![allow(clippy::upper_case_acronyms)]

/// Represents the register addresses in the NAU7802 ADC.
#[derive(Clone, Copy)]
#[repr(u8)]
pub(crate) enum Register {
    /// Power-Up Control Register
    PuCtrl = 0x00,
    /// Control Register 1
    Ctrl1,
    /// Control Register 2
    Ctrl2,
    /// Channel 1 Offset Calibration MSB
    Ocal1B2,
    /// Channel 1 Offset Calibration Middle Byte
    Ocal1B1,
    /// Channel 1 Offset Calibration LSB
    Ocal1B0,
    /// Channel 1 Gain Calibration MSB
    Gcal1B3,
    /// Channel 1 Gain Calibration Byte 2
    Gcal1B2,
    /// Channel 1 Gain Calibration Byte 1
    Gcal1B1,
    /// Channel 1 Gain Calibration LSB
    Gcal1B0,
    /// Channel 2 Offset Calibration MSB
    Ocal2B2,
    /// Channel 2 Offset Calibration Middle Byte
    Ocal2B1,
    /// Channel 2 Offset Calibration LSB
    Ocal2B0,
    /// Channel 2 Gain Calibration MSB
    Gcal2B3,
    /// Channel 2 Gain Calibration Byte 2
    Gcal2B2,
    /// Channel 2 Gain Calibration Byte 1
    Gcal2B1,
    /// Channel 2 Gain Calibration LSB
    Gcal2B0,
    /// I2C Control Register
    I2CControl,
    /// ADC Conversion Result MSB
    AdcoB2,
    /// ADC Conversion Result Middle Byte
    AdcoB1,
    /// ADC Conversion Result LSB
    AdcoB0,
    /// ADC Register (shared with OTP[32:24])
    Adc = 0x15,
    /// OTP Value Byte 1 (23:16 or 7:0 depending on mode)
    OtpB1,
    /// OTP Value Byte 0 (15:8)
    OtpB0,
    /// PGA Configuration Register
    Pga = 0x1B,
    /// PGA Power Configuration Register
    PgaPwr = 0x1C,
    /// Device Revision Code Register
    DeviceRev = 0x1F,
}

/// Trait for register bit manipulation.
pub(crate) trait RegisterBit {
    fn mask(&self) -> u8;
}

/// Bit definitions for the `PuCtrl` (Power-Up Control) register.
#[derive(Clone, Copy)]
#[repr(u8)]
pub(crate) enum PuCtrl {
    /// Reset all registers except `RR`.
    RR = 0,
    /// Power up digital circuit.
    PUD,
    /// Power up analog circuit.
    PUA,
    /// Power-up ready (Read-Only Status).
    PUR,
    /// Start a conversion cycle.
    CS,
    /// Cycle ready (ADC Data Ready; Read-Only).
    CR,
    /// Select system clock source: 0 = Internal RC, 1 = External Crystal.
    OSCS,
    /// Select AVDD source: 0 = External input, 1 = Internal LDO.
    AVDDS,
}

impl RegisterBit for PuCtrl {
    fn mask(&self) -> u8 {
        1 << *self as u8
    }
}

/// Bit definitions for the `Pga` (Programmable Gain Amplifier) register.
#[derive(Clone, Copy)]
#[repr(u8)]
pub(crate) enum Pga {
    /// Disable chopper stabilization.
    ChpDis = 0,
    /// Invert PGA input phase.
    Inv = 3,
    /// Enable PGA bypass.
    BypassEn,
    /// Enable PGA output buffer.
    OutEn,
    /// Select LDO mode: Improved stability or accuracy.
    LdoMode,
    /// Read OTP or ADC registers (shared access).
    RdOtpSel,
}

impl RegisterBit for Pga {
    fn mask(&self) -> u8 {
        1 << *self as u8
    }
}

/// Bit definitions for the `PgaPwr` (PGA Power Control) register.
#[derive(Clone, Copy)]
#[repr(u8)]
pub(crate) enum PgaPwr {
    /// Adjust PGA current (percentage of master bias current).
    Curr = 0,
    /// Adjust ADC current (percentage of master bias current).
    AdcCurr = 2,
    /// Adjust master bias current (percentage of nominal value).
    MstrBiasCurr = 4,
    /// Enable PGA output bypass capacitor.
    CapEn = 7,
}

impl RegisterBit for PgaPwr {
    fn mask(&self) -> u8 {
        1 << *self as u8
    }
}

/// Bit definitions for the `Ctrl2` (Control Register 2).
#[derive(Clone, Copy)]
#[repr(u8)]
pub(crate) enum Ctrl2 {
    /// Calibration mode: Offset or Gain calibration.
    CalMod = 0,
    /// Start a calibration.
    Cals = 2,
    /// Calibration error flag (Read-Only).
    CalError = 3,
    /// Conversion rate select.
    Crs = 4,
    /// Select analog input channel: 0 = Channel 1, 1 = Channel 2.
    Chs = 7,
}

impl RegisterBit for Ctrl2 {
    fn mask(&self) -> u8 {
        1 << *self as u8
    }
}
