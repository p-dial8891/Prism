use btleplug::api::{bleuuid::uuid_from_u16, Central, Manager as _, Peripheral as _, ScanFilter, WriteType};
use btleplug::platform::{Adapter, Manager, Peripheral};
use std::error::Error;
use std::time::Duration;
use tokio::time;
use uuid::Uuid;
use futures_util::StreamExt;

async fn find_prism(central: &Adapter) -> Option<Peripheral> {
    for p in central.peripherals().await.unwrap() {
        if p.properties()
            .await
            .unwrap()
            .unwrap()
            .local_name
            .iter()
            .any(|name| name.contains("Nordic_Blinky"))
        {
            return Some(p);
        }
    }
    None
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let manager = Manager::new().await.unwrap();

    // get the first bluetooth adapter
    let adapters = manager.adapters().await?;
    let central = adapters.into_iter().nth(0).unwrap();

    // start scanning for devices
    central.start_scan(ScanFilter::default()).await?;
    // instead of waiting, you can use central.events() to get a stream which will
    // notify you of new devices, for an example of that see examples/event_driven_discovery.rs
    time::sleep(Duration::from_secs(2)).await;

    // find the device we're interested in
    let prism = find_prism(&central).await.unwrap();

    // connect to the device
    prism.connect().await?;

    // discover services and characteristics
    prism.discover_services().await?;
    let chars = prism.characteristics();
/*
    // find the characteristic we want
	let KEY_CHAR_UUID : uuid::Uuid = Uuid::try_parse_ascii(b"00006d19-a468-4b3e-99b0-7c1f23675454")
	.unwrap();
    let key_char = chars.iter().find(|c| c.uuid == KEY_CHAR_UUID ).unwrap();
*/
    let SEED_CHAR_UUID = Uuid::try_parse_ascii(b"00006d18-a468-4b3e-99b0-7c1f23675454")
	.unwrap();
    let seed_char = chars.iter().find(|c| c.uuid == SEED_CHAR_UUID ).
	unwrap();
	prism.subscribe( seed_char ).await?;
	let mut prism_streams = prism.notifications().await?;

    loop 
	{ 
		if let Some(seed) = prism_streams.next().await
	    {
		    println!("{0}",seed.value.iter().nth(0).unwrap());
	    }
	};
}
