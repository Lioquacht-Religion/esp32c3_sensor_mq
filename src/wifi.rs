use anyhow::{anyhow, Result};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    wifi::{AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi},
};
use log::{error, info};

pub fn wifi<'m>(
    ssid: &str,
    pass: &str,
    modem: &'m mut esp_idf_svc::hal::modem::Modem,
    sysloop: EspSystemEventLoop,
) -> Result<(Box<EspWifi<'m>>, Vec<anyhow::Error>), anyhow::Error> {
    let mut auth_method = AuthMethod::WPA2Personal;
    if ssid.is_empty() {
        return Err(anyhow!("Missing WiFi name"));
    }
    if pass.is_empty() {
        auth_method = AuthMethod::None;
        info!("Wifi password is empty");
    }
    let mut esp_wifi = EspWifi::new(&mut *modem, sysloop.clone(), None).map_err(|e| anyhow!(e))?;
    // Connect to the Wi-Fi network
    const WIFI_CONNECT_RETRIES: usize = 10;
    let mut wifi_connect_tries = 0;
    let mut connect_errors: Vec<anyhow::Error> = Vec::new();
    loop {
        match try_wifi_connect(ssid, pass, sysloop.clone(), &mut esp_wifi, auth_method) {
            Ok(()) => return Ok((Box::new(esp_wifi), connect_errors)),
            Err(e) => {
                error!("wifi connection error: {}", e);
                if wifi_connect_tries >= WIFI_CONNECT_RETRIES {
                    return Err(e);
                }
                connect_errors.push(e);
            }
        }
        wifi_connect_tries += 1;
    }
}

pub fn try_wifi_connect<'m>(
    ssid: &str,
    pass: &str,
    sysloop: EspSystemEventLoop,
    esp_wifi: &mut EspWifi<'m>,
    auth_method: AuthMethod,
) -> Result<()> {
    let mut wifi = BlockingWifi::wrap(esp_wifi, sysloop)?;

    wifi.set_configuration(&Configuration::Client(ClientConfiguration::default()))?;

    info!("Starting wifi...");

    wifi.start()?;

    info!("Scanning...");

    let ap_infos = wifi.scan()?;

    let ours = ap_infos.into_iter().find(|a| a.ssid == ssid);

    let channel = if let Some(ours) = ours {
        info!(
            "Found configured access point {} on channel {}",
            ssid, ours.channel
        );
        Some(ours.channel)
    } else {
        info!(
            "Configured access point {} not found during scanning, will go with unknown channel",
            ssid
        );
        None
    };

    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: ssid
            .try_into()
            .expect("Could not parse the given SSID into WiFi config"),
        password: pass
            .try_into()
            .expect("Could not parse the given password into WiFi config"),
        channel,
        auth_method,
        ..Default::default()
    }))?;

    info!("Connecting wifi...");

    wifi.connect()?;

    info!("Waiting for DHCP lease...");

    wifi.wait_netif_up()?;

    let ip_info = wifi.wifi().sta_netif().get_ip_info()?;

    info!("Wifi DHCP info: {:?}", ip_info);

    Ok(())
}
