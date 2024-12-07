//! # Nuvoton NAU7802 Driver

#![cfg_attr(not(test), no_std)]

use crate::registers::*;
use byteorder::{BigEndian, ByteOrder};
use core::result::Result;
use embedded_hal_async::{delay, digital, i2c};

#[cfg(feature = "defmt")]
use defmt::*;

mod registers;

/// Default I2C device address.
pub const DEFAULT_DEVICE_ADDRESS: u8 = 0x2A;

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Voltage {
    L2v4 = 0b111,
    L2v7 = 0b110,
    L3v0 = 0b101,
    L3v3 = 0b100,
    L3v6 = 0b011,
    L3v9 = 0b010,
    L4v2 = 0b001,
    L4v5 = 0b000,
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Gain {
    G128 = 0b111,
    G64 = 0b110,
    G32 = 0b101,
    G16 = 0b100,
    G8 = 0b011,
    G4 = 0b010,
    G2 = 0b001,
    G1 = 0b000,
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum SamplesPerSecond {
    SPS320 = 0b111,
    SPS80 = 0b011,
    SPS40 = 0b010,
    SPS20 = 0b001,
    SPS10 = 0b000,
}

/// Errors that can occur when using the NAU7802 driver.
#[derive(PartialEq, Eq, Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<I2cErr, PinErr> {
    I2c(I2cErr),
    Pin(PinErr),
    PowerupFailed,
    CalibrationFailed,
    Timeout,
}

/// NAU7802 async driver.
pub struct Nau7802<D, DRDY, DELAY, const DEVICE_ADDRESS: u8 = DEFAULT_DEVICE_ADDRESS>
where
    D: i2c::I2c,
    DRDY: digital::Wait,
    DELAY: delay::DelayNs,
{
    i2c_dev: D,
    drdy_pin: Option<DRDY>,
    delay: DELAY,
}

impl<D, DRDY, DELAY, const DEVICE_ADDRESS: u8, I2cErr, PinErr>
    Nau7802<D, DRDY, DELAY, DEVICE_ADDRESS>
where
    D: i2c::I2c<Error = I2cErr>,
    DRDY: digital::Wait<Error = PinErr>,
    DELAY: delay::DelayNs,
{
    /// Creates a new NAU7802 driver instance.
    ///
    /// # Arguments
    /// i2c_dev: The I2C device.
    /// drdy_pin: The data ready pin. If not used, set to `None`.
    /// delay: The delay provider.
    /// gain: The instrumentation amplifier gain.
    /// sample_rate: The ADC sample rate.
    /// ldo_voltage: Optional internal LDO voltage.
    ///
    pub async fn try_new(
        i2c_dev: D,
        drdy_pin: Option<DRDY>,
        delay: DELAY,
        gain: Gain,
        sample_rate: SamplesPerSecond,
        ldo_voltage: Option<Voltage>,
    ) -> Result<Self, Error<I2cErr, PinErr>> {
        let mut adc = Self {
            i2c_dev,
            drdy_pin,
            delay,
        };
        adc.reset().await?;
        adc.power_up().await?;
        adc.set_gain(gain).await?;
        adc.set_sample_rate(sample_rate).await?;
        if let Some(ldo_voltage) = ldo_voltage {
            adc.set_ldo_voltage(ldo_voltage).await?;
        }
        adc.calibrate().await?;
        Ok(adc)
    }

    /// Reads the next ADC sample.
    pub async fn read(&mut self) -> Result<i32, Error<I2cErr, PinErr>> {
        // Wait for data ready
        if let Some(drdy) = &mut self.drdy_pin {
            drdy.wait_for_high().await.map_err(Error::Pin)?;
        } else {
            // Poll the CR bit if DRDY pin is not used
            self.poll_for_data_ready().await?;
        }

        // Read the ADC value registers.
        let mut buf = [0u8; 3];
        self.read_registers(Register::AdcoB2, &mut buf).await?;

        Ok(BigEndian::read_i24(&buf))
    }

    // Set the ADC sample rate.
    pub async fn set_sample_rate(
        &mut self,
        sample_rate: SamplesPerSecond,
    ) -> Result<(), Error<I2cErr, PinErr>> {
        const CRS_MASK: u8 = 0b10001111;
        let mut ctrl2 = self.read_register(Register::Ctrl2).await?;
        ctrl2 = (ctrl2 & CRS_MASK) | ((sample_rate as u8) << 4);
        self.write_register(Register::Ctrl2, ctrl2).await?;
        Ok(())
    }

    /// Sets the instrumentation amplifier gain.
    pub async fn set_gain(&mut self, gain: Gain) -> Result<(), Error<I2cErr, PinErr>> {
        const GAINS_MASK: u8 = 0b11111000;
        let mut ctrl1 = self.read_register(Register::Ctrl1).await?;
        ctrl1 = (ctrl1 & GAINS_MASK) | gain as u8;
        self.write_register(Register::Ctrl1, ctrl1).await?;
        Ok(())
    }

    /// Sets the LDO voltage (used for the ADC and analog circuitry).
    pub async fn set_ldo_voltage(
        &mut self,
        ldo_voltage: Voltage,
    ) -> Result<(), Error<I2cErr, PinErr>> {
        // Set the LDO voltage in the CTRL1 register.
        let mut ctrl1 = self.read_register(Register::Ctrl1).await?;
        ctrl1 = (ctrl1 & 0b11000111) | ((ldo_voltage as u8) << 3);
        self.write_register(Register::Ctrl1, ctrl1).await?;

        // Enable the internal LDO.
        self.write_register_bit(Register::PuCtrl, PuCtrl::AVDDS, true)
            .await?;

        Ok(())
    }

    /// Enables the PGA output bypass capacitor for improved ENOB in single channel applications.
    /// Requires an external capacitor be connected across VIN2P and VIN2N,
    /// See application circuit note 9.4.
    pub async fn pga_output_bypass_cap_enable(&mut self) -> Result<(), Error<I2cErr, PinErr>> {
        self.write_register_bit(Register::PgaPwr, PgaPwr::CapEn, true)
            .await
    }

    /// When interrupt pin is not used, poll for a conversion to complete.
    async fn poll_for_data_ready(&mut self) -> Result<(), Error<I2cErr, PinErr>> {
        // Timeout after ~150ms.
        for _ in 0..150 {
            let pu_ctrl = self.read_register(Register::PuCtrl).await?;
            if pu_ctrl & PuCtrl::CR.mask() != 0 {
                return Ok(());
            }
            self.delay.delay_ms(1).await;
        }
        Err(Error::Timeout)
    }

    /// Starts the internal calibration process.
    async fn calibrate(&mut self) -> Result<(), Error<I2cErr, PinErr>> {
        // Start calibration.
        self.write_register_bit(Register::Ctrl2, Ctrl2::Cals, true)
            .await?;

        // Poll for calibration completion.
        // TODO: how long should we actually wait?
        for _ in 0..1000 {
            let ctrl2 = self.read_register(Register::Ctrl2).await?;
            if ctrl2 & Ctrl2::Cals.mask() == 0 {
                if ctrl2 & Ctrl2::CalError.mask() != 0 {
                    return Err(Error::CalibrationFailed);
                }
                return Ok(());
            }
            self.delay.delay_ms(1).await;
        }
        Err(Error::Timeout)
    }

    /// Powers up the NAU7802 device.
    async fn power_up(&mut self) -> Result<(), Error<I2cErr, PinErr>> {
        // Get the current power-up control register value.
        let mut pu_ctrl = self.read_register(Register::PuCtrl).await?;

        // Enable digital and analog power-up (and clear power-up ready bit).
        pu_ctrl |= PuCtrl::PUD.mask() | PuCtrl::PUA.mask();
        pu_ctrl &= !PuCtrl::PUR.mask();

        self.write_register(Register::PuCtrl, pu_ctrl).await?;

        // Should take about 200us to power up.
        self.delay.delay_us(300).await;

        let pu_ctrl = self.read_register(Register::PuCtrl).await?;
        if pu_ctrl & PuCtrl::PUR.mask() == 0 {
            return Err(Error::PowerupFailed);
        }

        // Turn off CLK_CHP. From 9.1 power on sequencing.
        const TURN_OFF_CLK_CHP: u8 = 0x30;
        self.write_register(Register::Adc, TURN_OFF_CLK_CHP).await?;

        Ok(())
    }

    /// Performs a soft reset of the NAU7802 device.
    async fn reset(&mut self) -> Result<(), Error<I2cErr, PinErr>> {
        self.write_register_bit(Register::PuCtrl, PuCtrl::RR, true)
            .await?;
        self.write_register_bit(Register::PuCtrl, PuCtrl::RR, false)
            .await?;
        Ok(())
    }

    /// Writes a single bit in a register.
    async fn write_register_bit<T: RegisterBit>(
        &mut self,
        reg: Register,
        bit: T,
        enabled: bool,
    ) -> Result<(), Error<I2cErr, PinErr>> {
        let mut value = self.read_register(reg).await?;
        if enabled {
            value |= bit.mask();
        } else {
            value &= !bit.mask();
        }
        self.write_register(reg, value).await
    }

    /// Reads a single register from the NAU7802.
    async fn read_register(&mut self, reg: Register) -> Result<u8, Error<I2cErr, PinErr>> {
        let mut buf = [0u8];
        self.i2c_dev
            .write_read(DEVICE_ADDRESS, &[reg as u8], &mut buf)
            .await
            .map_err(Error::I2c)?;
        Ok(buf[0])
    }

    /// Reads multiple registers starting from a specific address.
    async fn read_registers(
        &mut self,
        start_reg: Register,
        buf: &mut [u8],
    ) -> Result<(), Error<I2cErr, PinErr>> {
        self.i2c_dev
            .write_read(DEVICE_ADDRESS, &[start_reg as u8], buf)
            .await
            .map_err(Error::I2c)
    }

    /// Writes a value to a single register.
    async fn write_register(
        &mut self,
        reg: Register,
        value: u8,
    ) -> Result<(), Error<I2cErr, PinErr>> {
        self.i2c_dev
            .write(DEVICE_ADDRESS, &[reg as u8, value])
            .await
            .map_err(Error::I2c)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::common::Generic;
    use embedded_hal_mock::eh1::{
        delay::NoopDelay,
        digital::{Mock as MockPin, State as PinState, Transaction as PinTransaction},
        i2c::{Mock as MockI2c, Transaction as I2cTransaction},
    };
    use std::sync::LazyLock;

    static I2C_SETUP_EXPECTATIONS: LazyLock<Vec<I2cTransaction>> = LazyLock::new(|| {
        vec![
            // Reset sequence.
            // Set RR bit.
            I2cTransaction::write_read(
                DEFAULT_DEVICE_ADDRESS,
                vec![Register::PuCtrl as u8],
                vec![0x00],
            ),
            I2cTransaction::write(
                DEFAULT_DEVICE_ADDRESS,
                vec![Register::PuCtrl as u8, PuCtrl::RR.mask()],
            ),
            // Clear RR bit.
            I2cTransaction::write_read(
                DEFAULT_DEVICE_ADDRESS,
                vec![Register::PuCtrl as u8],
                vec![0x00],
            ),
            I2cTransaction::write(DEFAULT_DEVICE_ADDRESS, vec![Register::PuCtrl as u8, 0x00]),
            // Power-up sequence.
            // Read PuCtrl.
            I2cTransaction::write_read(
                DEFAULT_DEVICE_ADDRESS,
                vec![Register::PuCtrl as u8],
                vec![0x00],
            ),
            // Set PUD and PUA bits.
            I2cTransaction::write(
                DEFAULT_DEVICE_ADDRESS,
                vec![
                    Register::PuCtrl as u8,
                    PuCtrl::PUD.mask() | PuCtrl::PUA.mask(),
                ],
            ),
            // Read PuCtrl.
            I2cTransaction::write_read(
                DEFAULT_DEVICE_ADDRESS,
                vec![Register::PuCtrl as u8],
                vec![PuCtrl::PUR.mask()],
            ),
            // Turn off CLK_CHP.
            I2cTransaction::write(DEFAULT_DEVICE_ADDRESS, vec![Register::Adc as u8, 0x30]),
            // Gain configuration.
            I2cTransaction::write_read(
                DEFAULT_DEVICE_ADDRESS,
                vec![Register::Ctrl1 as u8],
                vec![0x00],
            ),
            I2cTransaction::write(
                DEFAULT_DEVICE_ADDRESS,
                vec![Register::Ctrl1 as u8, Gain::G128 as u8],
            ),
            // Sample rate configuration.
            I2cTransaction::write_read(
                DEFAULT_DEVICE_ADDRESS,
                vec![Register::Ctrl2 as u8],
                vec![0x00],
            ),
            I2cTransaction::write(
                DEFAULT_DEVICE_ADDRESS,
                vec![Register::Ctrl2 as u8, (SamplesPerSecond::SPS10 as u8) << 4],
            ),
            // LDO voltage configuration.
            I2cTransaction::write_read(
                DEFAULT_DEVICE_ADDRESS,
                vec![Register::Ctrl1 as u8],
                vec![0x00],
            ),
            I2cTransaction::write(
                DEFAULT_DEVICE_ADDRESS,
                vec![Register::Ctrl1 as u8, (Voltage::L3v3 as u8) << 3],
            ),
            I2cTransaction::write_read(
                DEFAULT_DEVICE_ADDRESS,
                vec![Register::PuCtrl as u8],
                vec![0x00],
            ),
            I2cTransaction::write(
                DEFAULT_DEVICE_ADDRESS,
                vec![Register::PuCtrl as u8, PuCtrl::AVDDS.mask()],
            ),
            // Calibration.
            I2cTransaction::write_read(
                DEFAULT_DEVICE_ADDRESS,
                vec![Register::Ctrl2 as u8],
                vec![0x00],
            ),
            I2cTransaction::write(
                DEFAULT_DEVICE_ADDRESS,
                vec![Register::Ctrl2 as u8, Ctrl2::Cals.mask()],
            ),
            I2cTransaction::write_read(
                DEFAULT_DEVICE_ADDRESS,
                vec![Register::Ctrl2 as u8],
                vec![Ctrl2::Cals.mask()],
            ),
            I2cTransaction::write_read(
                DEFAULT_DEVICE_ADDRESS,
                vec![Register::Ctrl2 as u8],
                vec![0x00],
            ),
        ]
    });

    #[tokio::test]
    async fn test_try_new() {
        let mut mock_i2c = MockI2c::new(&*I2C_SETUP_EXPECTATIONS);

        let _driver: Nau7802<Generic<I2cTransaction>, Generic<PinTransaction>, NoopDelay> =
            Nau7802::try_new(
                mock_i2c.clone(),
                None,
                NoopDelay,
                Gain::G128,
                SamplesPerSecond::SPS10,
                Some(Voltage::L3v3),
            )
            .await
            .unwrap();

        mock_i2c.done();
    }

    #[tokio::test]
    async fn test_read_with_interrupt() {
        let mut i2c_expectations: Vec<I2cTransaction> = vec![];
        i2c_expectations.extend_from_slice(&*I2C_SETUP_EXPECTATIONS);

        // Read ADC value.
        i2c_expectations.push(I2cTransaction::write_read(
            DEFAULT_DEVICE_ADDRESS,
            vec![Register::AdcoB2 as u8],
            vec![0x1E, 0x84, 0x00],
        ));

        let pin_expectations = [
            // Wait for DRDY.
            PinTransaction::wait_for_state(PinState::High),
        ];

        let mut mock_i2c = MockI2c::new(&i2c_expectations);
        let mut mock_pin = MockPin::new(&pin_expectations);

        let mut driver: Nau7802<Generic<I2cTransaction>, Generic<PinTransaction>, NoopDelay> =
            Nau7802::try_new(
                mock_i2c.clone(),
                Some(mock_pin.clone()),
                NoopDelay,
                Gain::G128,
                SamplesPerSecond::SPS10,
                Some(Voltage::L3v3),
            )
            .await
            .unwrap();

        assert_eq!(driver.read().await.unwrap(), 1_999_872);

        mock_i2c.done();
        mock_pin.done();
    }

    #[tokio::test]
    async fn test_read_without_interrupt() {
        let mut i2c_expectations: Vec<I2cTransaction> = vec![];
        i2c_expectations.extend_from_slice(&*I2C_SETUP_EXPECTATIONS);

        // Wait for DRDY.
        for _ in 0..3 {
            i2c_expectations.push(I2cTransaction::write_read(
                DEFAULT_DEVICE_ADDRESS,
                vec![Register::PuCtrl as u8],
                vec![0b1110],
            ));
        }

        // Set DRDY.
        i2c_expectations.push(I2cTransaction::write_read(
            DEFAULT_DEVICE_ADDRESS,
            vec![Register::PuCtrl as u8],
            vec![0b1110 | PuCtrl::CR.mask()],
        ));

        // Read ADC value.
        i2c_expectations.push(I2cTransaction::write_read(
            DEFAULT_DEVICE_ADDRESS,
            vec![Register::AdcoB2 as u8],
            vec![0x1E, 0x84, 0x00],
        ));

        let mut mock_i2c = MockI2c::new(&i2c_expectations);

        let mut driver: Nau7802<Generic<I2cTransaction>, Generic<PinTransaction>, NoopDelay> =
            Nau7802::try_new(
                mock_i2c.clone(),
                None,
                NoopDelay,
                Gain::G128,
                SamplesPerSecond::SPS10,
                Some(Voltage::L3v3),
            )
            .await
            .unwrap();

        assert_eq!(driver.read().await.unwrap(), 1_999_872);

        mock_i2c.done();
    }
}
