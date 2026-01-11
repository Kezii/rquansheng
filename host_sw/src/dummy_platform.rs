use log::info;
use rquansheng::radio_platform::RadioPlatform;



pub struct DummyPlatform;

impl RadioPlatform for DummyPlatform {
    fn eeprom_read(address: u16, data: &mut [u8]) -> Result<(), Self::EepromError> {
        info!("eeprom_read: 0x{:x} {:?}", address, data);
        Ok(())
    }
    
    type EepromError = std::io::Error;
    
    fn eeprom_write(address: u16, data: &[u8; 8]) -> Result<(), Self::EepromError> {
        info!("eeprom_write: 0x{:x} {:?}", address, data);
        Ok(())
    }
    
    fn backlight_on(&mut self) {
        info!("backlight_on");
    }
    
    fn backlight_off(&mut self) {
        info!("backlight_off");
    }
    
    fn flashlight_on(&mut self) {
        info!("flashlight_on");
    }
    
    fn flashlight_off(&mut self) {
        info!("flashlight_off");
    }
    
    fn audio_path_on(&mut self) {
        info!("audio_path_on");
    }
    
    fn audio_path_off(&mut self) {
        info!("audio_path_off");
    }
}