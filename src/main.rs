use clap::{Arg, Parser};
use std::{thread, time::Duration, io::Read, str};

const CRSF_SYNC: u8 = 0xC8;

#[derive(Debug)]
enum PacketTypes {
    GPS = 0x02,
    Vario = 0x07,
    BatterySensor = 0x08,
    BaroAlt = 0x09,
    Heartbeat = 0x0B,
    VideoTransmitter = 0x0F,
    LinkStatistics = 0x14,
    RcChannelsPacked = 0x16,
    Attitude = 0x1E,
    FlightMode = 0x21,
    DeviceInfo = 0x29,
    ConfigRead = 0x2C,
    ConfigWrite = 0x2D,
    RadioID = 0x3A,
}

#[derive(Parser, Debug)]
#[command(name = "CRSF Packet Parser")]
#[command(about = "A tool to parse CRSF packets", long_about = None)]
struct Args {
    #[arg(short = 'P', long, default_value = "/dev/ttyAMA0")]
    port: String,
    #[arg(short = 'b', long, default_value_t = 420000)]
    baud: u32,
}

fn crc8_dvb_s2(mut crc: u8, a: u8) -> u8 {
    crc ^= a;

    for _ in 0..8 {
        if crc & 0x80 != 0 {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc <<= 1;
        }
    }

    return crc & 0xFF
}

fn crc8_data (data: &[u8]) -> u8 {
    return data.iter().fold(0, |crc, &byte| crc8_dvb_s2(crc, byte));
}

fn crsf_validate_frame(frame: &[u8]) -> bool {
    return crc8_data(&frame[2..frame.len() - 1]) == frame[frame.len() - 1];
}

fn signed_byte(b: u8) -> i8 {
    if b >= 128 {
        return (b as i16 - 256) as i8;
    } else {
        return b as i8;
    }
}

fn handle_crsf_packet(ptype: u8, data: &[u8]) {
    match ptype {
        x if x == PacketTypes::GPS as u8 => {
            let lat = i32::from_be_bytes([data[3], data[4], data[5], data[6]]) as f32 / 1e7;
            let lon = i32::from_be_bytes([data[7], data[8], data[9], data[10]]) as f32 / 1e7;
            let gspd = i16::from_be_bytes([data[11], data[12]]) as f32 / 36.0;
            let hdg = i16::from_be_bytes([data[13], data[14]]) as f32 / 100.0;
            let alt = i16::from_be_bytes([data[15], data[16]]) as i32 - 1000;
            let sats = data[17];
            println!(
                "GPS: Pos={} {} GSpd={:.1}m/s Hdg={:.1} Alt={}m Sats={}",
                lat, lon, gspd, hdg, alt, sats
            );
        }
        x if x == PacketTypes::Vario as u8 => {
            let vspd = i16::from_be_bytes([data[3], data[4]]) as f32 / 10.0;
            println!("VSpd: {:.1}m/s", vspd);
        }
        x if x == PacketTypes::Attitude as u8 => {
            let pitch = i16::from_be_bytes([data[3], data[4]]) as f32 / 10000.0;
            let roll = i16::from_be_bytes([data[5], data[6]]) as f32 / 10000.0;
            let yaw = i16::from_be_bytes([data[7], data[8]]) as f32 / 10000.0;
            println!("Attitude: Pitch={:.2} Roll={:.2} Yaw={:.2} (rad)", pitch, roll, yaw);
        }
        x if x == PacketTypes::BaroAlt as u8 => {
            let alt = i32::from_be_bytes([data[3], data[4], data[5], data[6]]) as f32 / 100.0;
            println!("Baro Altitude: {:.2}m", alt);
        }
        x if x == PacketTypes::LinkStatistics as u8 => {
            let rssi1 = signed_byte(data[3]);
            let rssi2 = signed_byte(data[4]);
            let lq = data[5];
            let snr = signed_byte(data[6]);
            println!("RSSI={}/{}dBm LQ={:03}", rssi1, rssi2, lq);
        }
        x if x == PacketTypes::BatterySensor as u8 => {
            let vbat = i16::from_be_bytes([data[3], data[4]]) as f32 / 10.0;
            let curr = i16::from_be_bytes([data[5], data[6]]) as f32 / 10.0;
            let mah = (data[7] as u32) << 16 | (data[8] as u32) << 8 | data[9] as u32;
            let pct = data[10];
            println!("Battery: {:.2}V {:.1}A {}mAh {}%", vbat, curr, mah, pct);
        }
        _ => {
            println!("Unknown packet type: {:x}", ptype);
        }
    }
}

fn main() {
    let args: Args = Args::parse();
    let mut port = serialport::new(args.port, args.baud)
        .timeout(Duration::from_secs(2))
        .open()
        .expect("Failed to open serial port");
    let mut input_buffer: Vec<u8> = Vec::new();
    let mut read_buffer = [0u8; 1024];

    loop {
        match port.read(&mut read_buffer) {
            Ok(n) => input_buffer.extend_from_slice(&read_buffer[..n]),
            Err(_) => thread::sleep(Duration::from_millis(10)),
        }

        while input_buffer.len() > 2 {
            let expected_len = (input_buffer[1] + 2) as usize;

            if expected_len > 64 || expected_len < 4 {
                input_buffer.clear();
            } else if input_buffer.len() >= expected_len {
                let single_packet = input_buffer.drain(..expected_len).collect::<Vec<u8>>();
                
                if crsf_validate_frame(&single_packet) {
                    println!("CRC error: {:?}", single_packet);
                } else {
                    handle_crsf_packet(single_packet[2], &single_packet);
                }
            }
        }
    }
}