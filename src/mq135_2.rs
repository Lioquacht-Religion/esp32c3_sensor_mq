// mq135_2.rs

use libm::powf;
use log::info;
/// For details about the parameters below, see:
/// http://davidegironi.blogspot.com/2014/01/cheap-co2-meter-using-mq135-sensor-with.html
/// https://hackaday.io/project/3475-sniffing-trinket/log/12363-mq135-arduino-library
///

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

pub struct Mq135 {
    /// Sensor baseline resistance in clean air (R0).
    r0: f32,
    /// Load resistance (RL) connected to the sensor.
    rl: f32,
}

impl Default for Mq135 {
    fn default() -> Self {
        Self {
            r0: 76.63,
            rl: 20.0,
        }
    }
}

impl Mq135 {
    /// Creates a new MQ135 sensor driver instance.
    ///
    /// # Arguments
    /// * `rl` - The load resistance value in kilo-ohms (usually 10kΩ).
    ///
    /// # Note
    /// The `r0` value is set to a default 10.0 and needs to be calibrated in clean air.
    pub fn new(rl: f32) -> Self {
        Mq135 {
            rl,
            ..Default::default()
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
    pub fn calibrate_in_clean_air(&mut self, adc_value: u16) -> anyhow::Result<()> {
        //let voltage = adc_value as f32 / 4095.0 * 3.3;
        let voltage = adc_value as f32 / 4096.0 * 5.0;

        //let rs = self.rl * (3.3 - voltage) / voltage;
        let rs = self.rl * (5.0 - voltage) / voltage;
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
    pub fn read_gas_ppm(&mut self, gas: GasType, adc_value: u16) -> anyhow::Result<(f32, u32)> {
        let adc_value: u32 = adc_value.into();

        let voltage = adc_value as f32 / 4095.0 * 3.3;
        info!("voltage: {}", voltage);
        let rs = self.rl * (3.3 - voltage) / voltage;
        info!("rs: {}", rs);
        info!("r0: {}", self.r0);
        let ratio = rs / self.r0;
        info!("ratio: {}", ratio);
        let (a, b) = gas_constants(gas);
        info!("a: {a}; b: {b}");
        let powfv = powf(ratio, b);
        info!("powf: {powfv}");
        Ok((a * powf(ratio, b), adc_value))
    }

    /*
    @brief  Get the resistance of the sensor, ie. the measurement value
            Known issue: If the ADC resolution is not 10-bits, this will give
            back garbage values!

    @return The sensor resistance in kOhm
    */
    pub fn get_resistance(&self, adc_value: u32) -> f32 {
        let val: f32 = adc_value as f32;
        //return ((1023./val) - 1.)* self.rl;
        //return (val* 2500.0 / 4095.0) * self.rl;
        //return (95.0/val) * self.rl;
        return ((4095. / val) - 1.) * self.rl;
    }

    pub fn get_correction_factor(t: f32, h: f32) -> f32 {
        // Linearization of the temperature dependency curve under and above 20 degree C
        // below 20degC: fact = a * t * t - b * t - (h - 33) * d
        // above 20degC: fact = a * t + b * h + c
        // this assumes a linear dependency on humidity
        if t < 20.0 {
            return CORA * t * t - CORB * t + CORC - (h - 33.) * CORD;
        } else {
            return CORE * t + CORF * h + CORG;
        }
    }

    pub fn get_corrected_resistance(&self, adc_value: u32, t: f32, h: f32) -> f32 {
        return self.get_resistance(adc_value) / Self::get_correction_factor(t, h);
    }

    pub fn get_ppm(&self, adc_value: u32) -> f32 {
        return PARA * powf(self.get_resistance(adc_value) / self.r0, -PARB);
    }

    pub fn get_corrected_ppm(&self, adc_value: u32, t: f32, h: f32) -> f32 {
        return PARA
            * powf(
                self.get_corrected_resistance(adc_value, t, h) / self.r0,
                -PARB,
            );
    }

    /*
    @brief  Get the resistance RZero of the sensor for calibration purposes

    @return The sensor resistance RZero in kOhm
    */
    pub fn get_rzero(&self, adc_value: u32) -> f32 {
        return self.get_resistance(adc_value) * powf(ATMOCO2 / PARA, 1. / PARB);
    }

    /*
    @brief  Get the corrected resistance RZero of the sensor for calibration
            purposes

    @param[in] t  The ambient air temperature
    @param[in] h  The relative humidity

    @return The corrected sensor resistance RZero in kOhm
    */
    pub fn get_corrected_rzero(&self, adc_value: u32, t: f32, h: f32) -> f32 {
        return self.get_corrected_resistance(adc_value, t, h) * powf(ATMOCO2 / PARA, 1. / PARB);
    }
}

/// Parameters to model temperature and humidity dependence
const CORA: f32 = 0.00035;
const CORB: f32 = 0.02718;
const CORC: f32 = 1.39538;
const CORD: f32 = 0.0018;
const CORE: f32 = -0.003333333;
const CORF: f32 = -0.001923077;
const CORG: f32 = 1.130128205;

/// Parameters for calculating ppm of CO2 from sensor resistance
const PARA: f32 = 116.6020682;
const PARB: f32 = 2.769034857;

/// Atmospheric CO2 level for calibration purposes,
/// from "Globally averaged marine surface monthly mean data"
/// available at https://gml.noaa.gov/ccgg/trends/gl_data.html
const ATMOCO2: f32 = 426.89; // Global CO2 May 2025
