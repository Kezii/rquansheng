use embedded_hal::delay::DelayNs;

pub struct DummyDelay;

impl DelayNs for DummyDelay {
    fn delay_ns(&mut self, ns: u32) {
        std::thread::sleep(std::time::Duration::from_nanos(ns as u64));
    }
}
