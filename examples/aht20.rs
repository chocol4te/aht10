//! Linux i2c demo

use {
    aht20::*,
    embedded_hal::blocking::delay::DelayMs,
    linux_embedded_hal as hal,
    std::{env, process},
};

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        println!("usage: {} /dev/i2c-N", args[0]);
        process::exit(1);
    }

    let i2c = hal::I2cdev::new(&args[1]).unwrap();

    let mut dev = AHT20::new(i2c, hal::Delay).unwrap();

    loop {
        let (h, t) = dev.read().unwrap();

        println!(
            "relative humidity={0}%; temperature={1}C",
            h.rh(),
            t.celsius()
        );

        hal::Delay.delay_ms(1000u16);
    }
}
