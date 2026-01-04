use embedded_graphics::{
    pixelcolor::{BinaryColor, Rgb888},
    prelude::*,
};
use embedded_graphics_simulator::{
    BinaryColorTheme, OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent, Window,
    sdl2::Keycode,
};
use log::info;
use rquansheng::{
    bk4819::Bk4819Driver,
    bk4819_bitbang::{Bk4819, Bk4819Bus},
    dialer::Dialer,
    display::RenderingMgr,
    keyboard::{KeyEvent, QuanshengKey},
    radio::{ChannelConfig, RadioController},
};

fn main() -> Result<(), core::convert::Infallible> {
    env_logger::init();
    let mut display = SimulatorDisplay::<BinaryColor>::new(Size::new(128, 64));
    let output_settings = OutputSettingsBuilder::new()
        .theme(BinaryColorTheme::Custom {
            color_off: Rgb888::new(216, 127, 64),
            color_on: Rgb888::new(49, 22, 13),
        })
        .scale(4)
        .pixel_aspect_ratio(2, 3)
        .build();
    let mut window = Window::new("rQuansheng", &output_settings);

    let dummy_radio_bus = DummyRadioBus;

    let mut radio = RadioController::new(Bk4819Driver::new(Bk4819::new(dummy_radio_bus)));

    'main: loop {
        radio.render_display(&mut display).unwrap();
        window.update(&display);

        for event in window.events() {
            if let SimulatorEvent::Quit = event {
                break 'main;
            }

            radio.eat_keyboard_event(simulator_event_to_quansheng_key(event));
        }

        radio.poll_interrupts().ok();

        std::thread::sleep(std::time::Duration::from_millis(100));
    }

    Ok(())
}

pub fn simulator_event_to_quansheng_key(event: SimulatorEvent) -> Option<KeyEvent> {
    match event {
        SimulatorEvent::KeyDown { keycode, .. } => {
            keycode_to_quansheng_key(keycode).map(KeyEvent::KeyPressed)
        }
        SimulatorEvent::KeyUp { keycode, .. } => {
            keycode_to_quansheng_key(keycode).map(KeyEvent::KeyReleased)
        }
        _ => None,
    }
}

pub fn keycode_to_quansheng_key(keycode: Keycode) -> Option<QuanshengKey> {
    match keycode {
        Keycode::Num0 => Some(QuanshengKey::Num0),
        Keycode::Num1 => Some(QuanshengKey::Num1),
        Keycode::Num2 => Some(QuanshengKey::Num2),
        Keycode::Num3 => Some(QuanshengKey::Num3),
        Keycode::Num4 => Some(QuanshengKey::Num4),
        Keycode::Num5 => Some(QuanshengKey::Num5),
        Keycode::Num6 => Some(QuanshengKey::Num6),
        Keycode::Num7 => Some(QuanshengKey::Num7),
        Keycode::Num8 => Some(QuanshengKey::Num8),
        Keycode::Num9 => Some(QuanshengKey::Num9),
        Keycode::Menu => Some(QuanshengKey::Menu),
        Keycode::Up => Some(QuanshengKey::Up),
        Keycode::Down => Some(QuanshengKey::Down),
        Keycode::Escape => Some(QuanshengKey::Exit),
        Keycode::Asterisk => Some(QuanshengKey::Star),
        Keycode::F => Some(QuanshengKey::F),
        Keycode::Space => Some(QuanshengKey::Ptt),
        Keycode::F1 => Some(QuanshengKey::Side2),
        Keycode::F2 => Some(QuanshengKey::Side1),
        _ => None,
    }
}

pub struct DummyRadioBus;

impl Bk4819Bus for DummyRadioBus {
    type Error = ();

    fn write_reg(&mut self, reg: u8, value: u16) -> Result<(), Self::Error> {
        info!("write_reg: 0x{:x} 0x{:x}", reg, value);
        Ok(())
    }

    fn read_reg(&mut self, reg: u8) -> Result<u16, Self::Error> {
        info!("read_reg: 0x{:x}", reg);
        Ok(0)
    }

    fn write_reg_n(&mut self, reg: rquansheng::bk4819_n::Register) -> Result<(), Self::Error> {
        info!("write_reg_n: {:?}", reg);
        Ok(())
    }

    fn read_reg_n(
        &mut self,
        reg: rquansheng::bk4819_n::Register,
    ) -> Result<rquansheng::bk4819_n::Register, Self::Error> {
        info!("read_reg_n: {:?}", reg);
        Ok(reg)
    }
}
