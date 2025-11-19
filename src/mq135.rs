// mq135.rs

fn test_adc(){

    let peripherals = Peripherals::take().unwrap();

    let adc = AdcDriver::new(peripherals.adc1).unwrap();
    let adc_config = AdcChannelConfig{
        attenuation: DB_11,
        ..Default::default()
    };

    let mut adc_pin //: AdcChannelDriver<'_, AdcDriver<'_, ADC1>> 
        = 
        AdcChannelDriver::new(
        &adc, peripherals.pins.gpio3, &adc_config
    ).unwrap();


    let mut sensor = Mq135::new(
         10_000.0
    ); // RL load resistance in Ohms

    sensor.calibrate_in_clean_air(&adc, &mut adc_pin).unwrap();
    let (co2_ppm, _) = sensor.read_gas_ppm(GasType::CO2, &adc, &mut adc_pin).unwrap();

    println!("CO2 Concentration: {:.2} ppm", co2_ppm);
}

//use embedded_hal;
use std::borrow::Borrow;
use esp_idf_svc::hal::{adc::{attenuation::DB_11, oneshot::{config::AdcChannelConfig, AdcChannelDriver, AdcDriver}, Adc}, gpio::ADCPin, prelude::Peripherals};
use libm::powf;

/// Represents the types of gases that the MQ135 sensor can detect.
#[derive(Copy, Clone, Debug)]
pub enum GasType {
    /// Carbon Dioxide (CO2)
    CO2,
    /// Ammonia (NH3)
    NH3,
    /// Benzene
    Benzene,
    /// Smoke (general smoke detection)
    Smoke,
}

/// Returns empirical constants (a, b) for the gas concentration formula
/// for each gas type. These constants come from datasheet curves.
///
/// The formula used is: ppm = a * (Rs/R0)^b
fn gas_constants(gas: GasType) -> (f32, f32) {
    match gas {
        GasType::CO2 => (110.47, -2.862),
        GasType::NH3 => (102.2, -2.473),
        GasType::Benzene => (44.947, -3.445),
        GasType::Smoke => (26.572, -2.265),
    }
}

pub struct Mq135
{
    /// Sensor baseline resistance in clean air (R0).
    r0: f32,
    /// Load resistance (RL) connected to the sensor.
    rl: f32,
}

impl Mq135{
    /// Creates a new MQ135 sensor driver instance.
    ///
    /// # Arguments
    /// * `rl` - The load resistance value in kilo-ohms (usually 10kΩ).
    ///
    /// # Note
    /// The `r0` value is set to a default 10.0 and needs to be calibrated in clean air.
    pub fn new(rl: f32) -> Self {
        Mq135 {
            r0: 10.0, // Default value, must be calibrated
            rl,
        }
    }

    /// Calibrates the sensor in clean air environment.
    ///
    /// This method reads the current sensor resistance in clean air,
    /// computes the baseline resistance (R0), and stores it internally.
    ///
    /// # Returns
    /// * `Ok(())` on success.
    /// * `Err(Error)` if the ADC reading fails.
    pub fn calibrate_in_clean_air<
        'd,
        ADC: Adc, 
        PIN: ADCPin, 
        BrwdADC : Borrow<AdcDriver<'d, PIN::Adc>>
    >(
        &mut self, 
        adc: &AdcDriver<'d, ADC>,
        pin: &mut AdcChannelDriver<'d, PIN, BrwdADC>
    ) -> anyhow::Result<()> {
        let adc_value: u32 = adc.read(pin)?.into();
        let voltage = adc_value as f32 / 4095.0 * 3.3;
        let rs = self.rl * (3.3 - voltage) / voltage;
        self.r0 = rs / 3.6;
        Ok(())
    }

    /// Reads the gas concentration (in PPM) for a specific gas type.
    ///
    /// # Arguments
    /// * `gas` - The target gas type to measure.
    ///
    /// # Returns
    /// * `Ok(ppm)` where `ppm` is the parts-per-million value of the gas.
    /// * `Err(Error)` if the ADC reading fails.
    pub fn read_gas_ppm<
        'd,
        ADC: Adc, 
        PIN: ADCPin, 
        BrwdADC : Borrow<AdcDriver<'d, PIN::Adc>>
    >(
        &mut self, gas: GasType,
        adc: &AdcDriver<'d, ADC>,
        pin: &mut AdcChannelDriver<'d, PIN, BrwdADC>
    ) -> anyhow::Result<(f32, u32)> {
        let adc_value: u32 = adc.read(pin)?.into();

        let voltage = adc_value as f32 / 4095.0 * 3.3;
        let rs = self.rl * (3.3 - voltage) / voltage;
        let ratio = rs / self.r0;
        let (a, b) = gas_constants(gas);
        Ok((a * powf(ratio, b), adc_value))
    }
}


