use std::io::{self, Read, Write};
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use rquansheng::messages::HostBound;
use rquansheng::messages::RadioBound;
use rquansheng::messages::decode_line;
use rquansheng::messages::encode_line;

fn read_line_from_port(port: &mut dyn Read, max_len: usize) -> io::Result<Vec<u8>> {
    let mut out = Vec::with_capacity(max_len.min(256));
    let mut byte = [0u8; 1];
    while out.len() < max_len {
        match port.read(&mut byte) {
            Ok(0) => continue,
            Ok(1) => {
                out.push(byte[0]);
                if byte[0] == 0 {
                    return Ok(out);
                }
            }
            Ok(_) => unreachable!("read(1 byte) returned >1"),
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            Err(e) => return Err(e),
        }
    }
    Err(io::Error::new(
        io::ErrorKind::InvalidData,
        "UART line too long (missing '\\n')",
    ))
}

fn main() {
    let mut port = serialport::new("/dev/ttyUSB0", 38400)
        .timeout(Duration::from_millis(5000))
        .open()
        .expect("Failed to open port");

    let mut port_clone = port.try_clone().unwrap();

    let (tx, rx) = mpsc::channel();

    let handle = thread::spawn(move || {
        loop {
            let line = read_line_from_port(&mut port, 256).unwrap();
            let reply = decode_line::<HostBound>(&line).unwrap();
            println!("Reply: {:?}", reply);
            tx.send(reply).unwrap();
        }
    });

    let mut now = std::time::Instant::now();
    loop {
        let ping = RadioBound::ReadRegister(0x02);
        let ping_encoded = encode_line(&ping).unwrap();
        port_clone.write_all(&ping_encoded).unwrap();

        let reply = rx.recv().unwrap();
        let _ = rx.try_recv();

        let elapsed = now.elapsed();
        println!("messages per second: {}", 1.0 / elapsed.as_secs_f64());
        now = std::time::Instant::now();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rquansheng::messages::RadioBound;
    use rquansheng::messages::decode_line;
    use rquansheng::messages::encode_line;

    #[test]
    fn test_encode_decode() {
        let ping = RadioBound::Ping;
        let ping_encoded = encode_line(&ping).unwrap();
        let ping_decoded = decode_line::<RadioBound>(&ping_encoded).unwrap();
        assert_eq!(ping_decoded, ping);

        let write_register = RadioBound::WriteRegister(0x10, 0x20);
        let write_register_encoded = encode_line(&write_register).unwrap();
        let write_register_decoded = decode_line::<RadioBound>(&write_register_encoded).unwrap();
        assert_eq!(write_register_decoded, write_register);
    }
}
