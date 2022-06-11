#![no_std]
#![no_main]

use l3gd20::{L3gd20, Odr};
use lsm303dlhc::{AccelOdr, Lsm303dlhc, MagOdr};

use ahrs::Madgwick;
use ahrs::Ahrs;

use panic_halt as _;

use cortex_m_rt::entry;
use cortex_m_rt::exception;

use cortex_m::{asm};
use cortex_m::peripheral::SYST;

use stm32f3xx_hal::{self as hal, pac, time, prelude::*};
use stm32f3xx_hal::i2c::I2c;
use stm32f3xx_hal::spi::Spi;
use crate::pac::{I2C1, Peripherals};

use core::f32::consts::PI;
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_semihosting::hprintln;

use nalgebra::{Vector3, UnitQuaternion, Quaternion};


// Magnetometer calibration parameters
// NOTE you need to use the right parameters for *your* magnetometer
// You can use the `log-sensors` example to calibrate your magnetometer. The producer is explained
// in https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration
const M_BIAS_X: f32 = -34.;
const M_SCALE_X: f32 = 650.;

const M_BIAS_Y: f32 = -70.;
const M_SCALE_Y: f32 = 636.;

const M_BIAS_Z: f32 = -37.5;
const M_SCALE_Z: f32 = 589.5;

// Sensitivities of the accelerometer and gyroscope, respectively
const K_G: f32 = 2. / (1 << 15) as f32; // LSB -> g
const K_AR: f32 = 8.75e-3 * PI / 180.; // LSB -> rad/s

// Number of samples to use for gyroscope calibration
const NSAMPLES: i32 = 256;

static systick_reload: AtomicU32 = AtomicU32::new(0);

#[entry]
fn main() -> ! {
// Get access to the core peripherals from the cortex-m crate

    hprintln!("get_ticks_per_10ms {}", SYST::get_ticks_per_10ms());

    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();

    let clocks = rcc
        .cfgr
        .use_pll()
        .use_hse(8.MHz())
        .sysclk(72.MHz())
        .pclk1(24.MHz())
        .freeze(&mut flash.acr);

    let mut cp = cortex_m::Peripherals::take().unwrap();

    cp.SYST.set_clock_source(SystClkSource::External);
    cp.SYST.set_reload(0xFFFFFFFF);
    cp.SYST.clear_current();
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();

    hprintln!("get_ticks_per_10ms {} {} {:?}", SYST::get_ticks_per_10ms(), SYST::has_reference_clock(), clocks);

    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

    let mut led = gpioe.pe13.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

    /**************************************/

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let mut scl = gpiob.pb6.into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let mut sda = gpiob.pb7.into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

    scl.internal_pull_up(&mut gpiob.pupdr, true);
    sda.internal_pull_up(&mut gpiob.pupdr, true);

    let mut i2c = hal::i2c::I2c::new(
        dp.I2C1,
        (scl, sda),
        400000.Hz(),
        clocks,
        &mut rcc.apb1,
    );

    let mut lsm303dlhc = Lsm303dlhc::new(i2c).unwrap();
    lsm303dlhc.accel_odr(AccelOdr::Hz200).unwrap();
    lsm303dlhc.mag_odr(MagOdr::Hz220).unwrap();

    /**************************************/

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);

    let sck = gpioa.pa5.into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let miso = gpioa.pa6.into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let mosi = gpioa.pa7.into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);

    let mut nss = gpioe.pe3.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    nss.set_high();

    let mut spi = Spi::new(dp.SPI1, (sck, miso, mosi), 18.MHz(), clocks, &mut rcc.apb2);
    let mut l3gd20 = L3gd20::new(spi, nss).unwrap();

    l3gd20.set_odr(Odr::Hz190).unwrap();

    /**************************************/
/*
    let sensor_delay = 5;

    // Calibrate the gyroscope
    let mut ar_bias_x: i32 = 0;
    let mut ar_bias_y: i32 = 0;
    let mut ar_bias_z: i32 = 0;

    for _ in 0..NSAMPLES {
        asm::delay(8_000_000 / 200);

        let ar = l3gd20.gyro().unwrap();

        ar_bias_x += i32::from(ar.x);
        ar_bias_y += i32::from(ar.y);
        ar_bias_z += i32::from(ar.z);
    }
    let ar_bias_x = (ar_bias_x / NSAMPLES) as i16;
    let ar_bias_y = (ar_bias_y / NSAMPLES) as i16;
    let ar_bias_z = (ar_bias_z / NSAMPLES) as i16;
*/
    // Initialize filter with default values
    let mut ahrs = Madgwick::<f32>::new(1.0/200.0, 0.1);

    let mut loop_count = 0;

    let mut last = SYST::get_current();

    let mut sum_lsm303dlhc = 0;
    let mut sum_l3gd20 = 0;
    let mut sum_lsm303dlhc = 0;
    let mut sum_calib = 0;
    let mut sum_ahrs = 0;
    let mut sum_euler = 0;
    loop {

        loop_count += 1;

        let ms = SYST::get_current();
        let m = lsm303dlhc.mag().unwrap();
        sum_lsm303dlhc = diff_clock(SYST::get_current(), ms);

        let ms = SYST::get_current();
        let ar = l3gd20.gyro().unwrap();
        sum_l3gd20 = diff_clock(SYST::get_current(), ms);

        let ms = SYST::get_current();
        let g = lsm303dlhc.accel().unwrap();
        sum_lsm303dlhc = diff_clock(SYST::get_current(), ms);

        let ms = SYST::get_current();
        let m_x = (f32::from(m.x) - M_BIAS_X) / M_SCALE_X;
        let m_y = (f32::from(m.y) - M_BIAS_Y) / M_SCALE_Y;
        let m_z = (f32::from(m.z) - M_BIAS_Z) / M_SCALE_Z;

        // Fix the X Y Z components of the magnetometer so they match the gyro axes
        let magnetometer= Vector3::new (
            m_y,
            -m_x,
            m_z,
        );

        let ar_x = f32::from(ar.x) * K_AR;
        let ar_y = f32::from(ar.y) * K_AR;
        let ar_z = f32::from(ar.z) * K_AR;
        let accelerometer = Vector3::new (
            ar_x,
            ar_y,
            ar_z,
        );

        // Fix the X Y Z components of the accelerometer so they match the gyro axes
        let g_x = f32::from(g.x) * K_G;
        let g_y = f32::from(g.y) * K_G;
        let g_z = f32::from(g.z) * K_G;
        let gyroscope = Vector3::new(
            g_y,
            -g_x,
            g_z,
        );
        sum_calib = diff_clock(SYST::get_current(), ms);

        let ms = SYST::get_current();
        let quat = ahrs.update(
                &(gyroscope * (PI / 180.0)),
                &accelerometer,
                &magnetometer,
            )
            .unwrap();
        sum_ahrs = diff_clock(SYST::get_current(), ms);

        let ms = SYST::get_current();
        let (roll, pitch, yaw) = quat.euler_angles();
        sum_euler = diff_clock(SYST::get_current(), ms);

        let current = SYST::get_current();
        if diff_clock(current, last) >= 1_000 * 72{
            last = current;
            //hprintln!("{} {} - roll {} pitch {} yaw {}", systick_reload.load(Ordering::SeqCst), loop_count, roll, pitch, yaw);
            hprintln!("{} {}", systick_reload.load(Ordering::SeqCst), SYST::get_ticks_per_10ms());
            led.toggle().unwrap();

            hprintln!("{} {} {} {} {} {}",
                sum_lsm303dlhc,
                sum_l3gd20,
                sum_lsm303dlhc,
                sum_calib,
                sum_ahrs,
                sum_euler);

            sum_lsm303dlhc = 0;
            sum_l3gd20 = 0;
            sum_lsm303dlhc = 0;
            sum_calib = 0;
            sum_ahrs = 0;
            sum_euler = 0;
        }
        //hprintln!("loop {} {} {}", systick_reload.load(Ordering::SeqCst), SYST::get_current(), micros());
        //asm::delay(72_000_000/1000);

    }
}

fn diff_clock(a:u32, b:u32) -> u32 {
    b.wrapping_sub(a) & 0x00FF_FFFF
}

#[exception]
fn SysTick() -> () {
    systick_reload.fetch_add(1, Ordering::SeqCst);
}

// monitor arm semihosting enable
