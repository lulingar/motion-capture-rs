use anyhow::Result;
use esp_idf_svc::hal::{
    peripherals::Peripherals,
    units::FromValueType,
    i2c::{I2cConfig, I2cDriver},
    delay::FreeRtos,
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
    primitives::{PrimitiveStyle, Triangle},
};

//use rand::prelude::*;

fn main() -> Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Hello, world!");

    let peripherals = Peripherals::take().unwrap();
    let sda = peripherals.pins.gpio8;
    let scl = peripherals.pins.gpio9;
    let i2c = peripherals.i2c0;

    let config = I2cConfig::new().baudrate(400.kHz().into());
    let i2c = I2cDriver::new(i2c, sda, scl, &config)?;

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display).unwrap();

    Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
        .draw(&mut display).unwrap();

    let thin_stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let yoffset = 10;
    Triangle::new(
        Point::new(50, 25 + yoffset),
        Point::new(50 + 25, 25 + yoffset),
        Point::new(50 + 17, yoffset),
    )
    .into_styled(thin_stroke)
    .draw(&mut display).unwrap();

    display.flush().unwrap();

    loop {
        FreeRtos::delay_ms(2);
    }
}
