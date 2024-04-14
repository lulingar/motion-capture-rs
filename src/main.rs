use anyhow::Result;
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    nvs::EspDefaultNvsPartition,
    mqtt::client::*
};
use esp_idf_svc::hal::{
    delay::FreeRtos,
    gpio::PinDriver,
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
    units::FromValueType,
};
use esp_idf_svc::wifi::{BlockingWifi, EspWifi};
use embedded_svc::wifi::{AuthMethod, ClientConfiguration, Configuration};
use embedded_hal_bus::i2c;
use core::cell::RefCell;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_hal_bus::i2c::RefCellDevice;
use ssd1306::mode::BufferedGraphicsMode;

//use rand::prelude::*;

// The constant `CONFIG` is auto-generated by `toml_config`.
#[toml_cfg::toml_config]
pub struct Config {
    #[default("localhost")]
    mqtt_host: &'static str,
    #[default("1883")]
    mqtt_port: &'static str,
    #[default("")]
    mqtt_user: &'static str,
    #[default("")]
    mqtt_pass: &'static str,
    #[default("")]
    wifi_ssid: &'static str,
    #[default("")]
    wifi_psk: &'static str,
}

fn main() -> Result<()> {
    // It is necessary to call this function once. Otherwise, some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    let i2c_config = I2cConfig::new().baudrate(400.kHz().into());
    let sda_imu = peripherals.pins.gpio1;
    let scl_imu = peripherals.pins.gpio0;
    let i2c = I2cDriver::new(peripherals.i2c0, sda_imu, scl_imu, &i2c_config)?;
    let i2c_ref_cell = RefCell::new(i2c);  

    let mut flag_total = PinDriver::output(peripherals.pins.gpio21)?;
    let mut flag_compute = PinDriver::output(peripherals.pins.gpio20)?;
    flag_total.set_low()?;
    flag_compute.set_low()?;

    let interface = I2CDisplayInterface::new(i2c::RefCellDevice::new(&i2c_ref_cell));
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate180)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();
    
    let base_point = Point::zero();
    let msg = format!("SSID: {}", CONFIG.wifi_ssid);
    let mut base_point = display_msg(&mut display, text_style, msg, base_point);

    // Connect to the Wi-Fi network
    let mut wifi = BlockingWifi::wrap(
        EspWifi::new(peripherals.modem, sys_loop.clone(), Some(nvs.clone()))?,
        sys_loop.clone(),
    )?;
    connect_wifi(&mut wifi)?;
    let connected = wifi.is_connected().unwrap();
    base_point.x = 0;
    let msg = format!("Connected? {:?}", connected);
    let mut base_point = display_msg(&mut display, text_style, msg, base_point);

    if connected {
        let msg = format!("IP: {}", wifi.wifi().sta_netif().get_ip_info().unwrap().ip);
        base_point.x = 0;
        let mut _base_point = display_msg(&mut display, text_style, msg, base_point);
    }

    let mqtt_url: String = format!("mqtt://{}:{}", CONFIG.mqtt_host, CONFIG.mqtt_port);
    let (mut client, mut conn) = mqtt_create(&mqtt_url, CONFIG.mqtt_user, CONFIG.mqtt_pass)?;

    std::thread::Builder::new()
        .stack_size(6000)
        .spawn(move || {
            log::info!("MQTT Listening for messages");
            while let Ok(event) = conn.next() {
                log::info!("[Queue] Event: {}", event.payload());
            }
            log::info!("Connection closed");
        })
        .unwrap();
    
    const topic: &str = "raw";
    const payload: &str = "contents";
    loop {
        client.enqueue(topic, QoS::AtMostOnce, false, payload.as_bytes())?;
        log::info!("Published \"{payload}\" to topic \"{topic}\"");
        FreeRtos::delay_ms(2000);
    }
}

fn display_msg(display: &mut Ssd1306<I2CInterface<RefCellDevice<I2cDriver>>, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>,
               text_style: MonoTextStyle<BinaryColor>,
               msg: String,
               point: Point,
) -> Point {
    let t = Text::with_baseline(&msg, point, text_style, Baseline::Top);
    t.draw(display).unwrap();
    display.flush().unwrap();
    t.bounding_box().bottom_right().unwrap()
}


fn connect_wifi(wifi: &mut BlockingWifi<EspWifi<'static>>) -> Result<()> {
    let wifi_configuration: Configuration = Configuration::Client(ClientConfiguration {
        ssid: CONFIG.wifi_ssid.parse().unwrap(),
        password: CONFIG.wifi_psk.parse().unwrap(),
        auth_method: AuthMethod::WPAWPA2Personal,
        channel: None,
        ..Default::default()
    });
    wifi.set_configuration(&wifi_configuration)?;
    log::info!("CFG: {:?}", wifi_configuration);

    wifi.start()?;
    log::info!("Wifi started? {}", wifi.is_started()?);

    /*
    let caps = wifi.get_capabilities()?;
    log::info!("Capabilities {:?}", caps);

    if let Ok(net) = wifi.scan() {
        for ap in net {
            log::info!("Available: {:?}", ap);
        }
    }
    */

    wifi.connect()?;
    log::info!("Wifi connected");

    wifi.wait_netif_up()?;
    log::info!("Wifi netif up");

    Ok(())
}

fn mqtt_create(
    url: &str,
    client_id: &str,
    password: &str,
) -> Result<(EspMqttClient<'static>, EspMqttConnection)> {
    let (mqtt_client, mqtt_conn) = EspMqttClient::new(
        url,
        &MqttClientConfiguration {
            client_id: Some(client_id),
            password: Some(password),
            username: Some(client_id),
            ..Default::default()
        },
    )?;

    Ok((mqtt_client, mqtt_conn))
}