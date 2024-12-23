use clap::Parser;
use crossbeam_channel::{bounded, select, tick, unbounded, Receiver};
use crossterm::{
    cursor::*,
    event,
    event::poll,
    event::{Event, KeyCode, KeyEvent},
    execute,
    style::*,
    terminal::{disable_raw_mode, enable_raw_mode, size, Clear, ClearType},
    ExecutableCommand, QueueableCommand, Result,
};

use std::io::stdout;
use std::thread;
use std::time::Duration;
use msp::MSPMsg;
use msp::States;

mod msp;


#[derive(Parser, Debug)]
#[command(name = "CRSF Packet Parser")]
#[command(about = "A tool to parse CRSF packets", long_about = None)]
struct Args {
    #[arg(short = 'P', long, default_value = "/dev/ttyACM0")]
    port: String,
    #[arg(short = 'b', long, default_value_t = 115200)]
    baud: u32,
}

pub fn get_serial_devices(defdev: &str, testcvt: bool) -> String
{
    match serialport::available_ports() {
        Ok(ports) => {
            for p in ports {
                match &p.port_type {
                    serialport::SerialPortType::UsbPort(pt) => {
                        if (pt.vid == 0x0483 && pt.pid == 0x5740)
                            || (pt.vid == 0x0403 && pt.pid == 0x6001)
                            || (testcvt && (pt.vid == 0x10c4 && pt.pid == 0xea60))
                        {
                            return p.port_name.clone();
                        }
                    }
                    _ => {
                        if std::env::consts::OS == "freebsd" && &p.port_name[0..9] == "/dev/cuaU" {
                            return p.port_name.clone();
                        }
                    }
                }
            }

            return defdev.to_string();
        },
        Err(_e) => defdev.to_string()    
    }
}

fn wait_for_key (cc: &Receiver<u8>, tot: u64, itm: u32) -> bool
{
    let ticks = tick(Duration::from_millis(tot));
    let mut j = 0;
    loop {
        select! {
            recv(ticks) -> _ => {
                j += 1;
                if j == itm {
                    return true;
                }
            }
            recv(cc) -> res => {
                if let Ok(x) = res  {
                    if x == b'Q' { return false}
                }
            }
        }
    }
}

fn outvalue(y: u16, val: &str) -> Result<()>
{
    stdout()
        .execute(MoveTo(10, y))?
        .execute(SetAttribute(Attribute::Bold))?
        .execute(Print(val))?
        .execute(SetAttribute(Attribute::Reset))?
        .execute(Clear(ClearType::UntilNewLine))?;

    Ok(())
}

fn main() {
    let args: Args = Args::parse();
    let defdev= "auto";
    let _pname: String;
    let mut buffer  = [0u8; 256];
    let n = States::Init;

    _pname = get_serial_devices(defdev, true);

    let mut port: Box<dyn serialport::SerialPort> = serialport::new(_pname, args.baud)
        .timeout(Duration::from_millis(50))
        .open()
        .expect("Error open port!");

    loop {
        let payload: [u8; 0] = [];

        let request = msp::encode_msp(msp::MSG_BOARD_INFO, &payload);
        port.write_all(&request).expect("Err request send");

        let mut response = [0u8; 1024];
        match port.read(&mut response) {
            Ok(bytes_read) => {
                let board = if response.len() > 8 {
                    String::from_utf8_lossy(&response[9..])
                } else {
                    String::from_utf8_lossy(&response[0..4])
                };

                outvalue(10, &board).unwrap();
                // Тут потрібно розпарсити MSP-відповідь
            }
            Err(e) => eprintln!("Помилка читання: {}", e),
        }
    }
}