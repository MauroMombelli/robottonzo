#![no_std]
#![no_main]

use ahrs::Madgwick;
use cortex_m::asm;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use l3gd20::{L3gd20, Odr};
use lsm303dlhc::{AccelOdr, Lsm303dlhc, MagOdr};
use panic_halt as _;
use stm32f3xx_hal::{self as hal, pac, prelude::*};
use stm32f3xx_hal::i2c::I2c;
use stm32f3xx_hal::spi::Spi;
use stm32f3xx_hal::time;
use crate::pac::{I2C1, Peripherals};
//use embedded_hal::blocking::delay::DelayMs;
use cortex_m::peripheral::SYST;

use core::f32::consts::PI;
use core::sync::atomic::{AtomicU32, Ordering};
use ahrs::Ahrs;
use nalgebra::{Vector3, UnitQuaternion, Quaternion};

use cortex_m_rt::exception;


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

static counter: AtomicU32 = AtomicU32::new(0);

#[entry]
fn main() -> ! {
// Get access to the core peripherals from the cortex-m crate
    let mut cp = cortex_m::Peripherals::take().unwrap();

    cp.SYST.set_reload(SYST::get_ticks_per_10ms());
    cp.SYST.clear_current();
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();


    let dp = pac::Peripherals::take().unwrap();
    //let mut sys = dp.SYSCFG.constrain();
    //sys.set_re
    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(48.MHz())
        .pclk1(24.MHz())
        .freeze(&mut flash.acr);

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

    let mut spi = Spi::new(dp.SPI1, (sck, miso, mosi), 3.MHz(), clocks, &mut rcc.apb2);
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

    let mut last = counter.load(Ordering::SeqCst);
    loop {
        loop_count += 1;
        //hprintln!("Hello, world! {}", count).unwrap();


        let m = lsm303dlhc.mag().unwrap();
        let ar = l3gd20.gyro().unwrap();
        let g = lsm303dlhc.accel().unwrap();

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

        let quat = ahrs.update(
                &(gyroscope * (PI / 180.0)),
                &accelerometer,
                &magnetometer,
            )
            .unwrap();
        let (roll, pitch, yaw) = quat.euler_angles();

        let current = counter.load(Ordering::SeqCst);
        if current - last >= 100{
            last = current;
            hprintln!("{} {} - roll {} pitch {} yaw {}", counter.load(Ordering::SeqCst), loop_count, roll, pitch, yaw);
            led.toggle().unwrap();
        }
        //asm::delay(8_000_000/200);

    }
}

#[exception]
fn SysTick() -> () {
    counter.fetch_add(1, Ordering::SeqCst);
}

//monitor arm semihosting enable