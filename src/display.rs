use display_interface_spi::SPIInterface;
use dp32g030::{PORTCON, SPI0, SYSCON};
use embedded_graphics::{
    mono_font::{ascii::FONT_8X13_BOLD, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::Point,
    text::Text,
};
use embedded_hal_bus::spi::ExclusiveDevice;
use heapless::String;
use st7565::{
    types::{BoosterRatio, PowerControlMode},
    DisplaySpecs, GraphicsPageBuffer, ST7565,
};
use static_cell::StaticCell;

use crate::dp30g030_hal::{
    self,
    gpio::Port,
    spi::{self, SckPin},
};
use crate::{
    bk4819_bitbang::{Bk4819BitBang, Dp32g030BidiPin},
    delay::CycleDelay,
    dialer::Dialer,
    dp30g030_hal::gpio::{Output, Pin},
    radio::{ChannelConfig, RadioController},
};

use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::Drawable;

#[allow(non_camel_case_types)]
pub struct FG12864390_FKFW;
impl DisplaySpecs<128, 64, 8> for FG12864390_FKFW {
    const FLIP_ROWS: bool = false;
    const FLIP_COLUMNS: bool = true;
    const INVERTED: bool = false;
    const BIAS_MODE_1: bool = false;
    const POWER_CONTROL: PowerControlMode = PowerControlMode {
        booster_circuit: true,
        voltage_regulator_circuit: true,
        voltage_follower_circuit: true,
    };
    const VOLTAGE_REGULATOR_RESISTOR_RATIO: u8 = 0b100; // RR=4
    const ELECTRONIC_VOLUME: u8 = 31;
    const BOOSTER_RATIO: BoosterRatio = BoosterRatio::StepUp2x3x4x;
    const COLUMN_OFFSET: u8 = 4;
}

type DisplaySpiDevice = ExclusiveDevice<spi::Spi0, Pin<Output>, embedded_hal_bus::spi::NoDelay>;
type DisplayInterface = SPIInterface<DisplaySpiDevice, Pin<Output>>;
type Display = ST7565<
    DisplayInterface,
    FG12864390_FKFW,
    st7565::modes::GraphicsMode<'static, 128, 8>,
    128,
    64,
    8,
>;

static PAGE_BUFFER: StaticCell<GraphicsPageBuffer<128, 8>> = StaticCell::new();

pub struct Rendering {
    display: Display,
}

impl Rendering {
    pub fn new(spi0: SPI0, syscon: &SYSCON, portcon: &PORTCON) -> Self {
        // --- Display (ST7565) -----------------------------------------------------
        //
        // Wiring from the reference UV-K5 firmware:
        // - SPI0: PB8=CLK, PB10=MOSI (PB9 is used as A0/DC, so we run write-only, no MISO)
        // - CS:   PB7 (we manage it as GPIO; embedded-hal `SpiBus` is bus-only)
        // - A0/DC: PB9
        // - RST:  PB11 (shared with SWDIO in stock firmware)
        let spi0_sck = spi::SckPin::<dp30g030_hal::SPI0>::new(Pin::new(Port::B, 8)).unwrap();
        let spi0_mosi = spi::MosiPin::<dp30g030_hal::SPI0>::new(Pin::new(Port::B, 10)).unwrap();
        let spi0_cfg = spi::Config::uvk5_display_default();
        let spi0: spi::Spi0 = spi::Spi::<
            dp30g030_hal::SPI0,
            spi::SckPin<dp30g030_hal::SPI0>,
            spi::MosiPin<dp30g030_hal::SPI0>,
            spi::NoMiso,
        >::new(
            spi0,
            &syscon,
            &portcon,
            spi0_sck,
            spi0_mosi,
            spi::NoMiso,
            spi0_cfg,
        )
        .unwrap();

        let pin_lcd_cs = Pin::new(Port::B, 7).into_push_pull_output(&syscon, &portcon);
        let pin_lcd_a0 = Pin::new(Port::B, 9).into_push_pull_output(&syscon, &portcon);
        let mut pin_lcd_rst = Pin::new(Port::B, 11).into_push_pull_output(&syscon, &portcon);

        let mut disp_delay = CycleDelay::new(48_000_000);

        let disp_spidevice = ExclusiveDevice::new_no_delay(spi0, pin_lcd_cs).unwrap();
        let disp_interface = SPIInterface::new(disp_spidevice, pin_lcd_a0);

        let page_buffer = PAGE_BUFFER.init(GraphicsPageBuffer::new());
        let mut display: Display =
            ST7565::new(disp_interface, FG12864390_FKFW).into_graphics_mode(page_buffer);

        display.reset(&mut pin_lcd_rst, &mut disp_delay).ok();
        display.set_display_on(true).unwrap();
        display.flush().unwrap();

        Self { display }
    }

    pub fn render_main(&mut self, channel_cfg: ChannelConfig, rssi: f32, dialer: &Dialer) {
        let mut string = String::<64>::new();
        use core::fmt::Write;

        writeln!(string, "> {} kHz", dialer.get_as_string()).unwrap();
        writeln!(string, "f: {} kHz", channel_cfg.freq / 1000).unwrap();
        writeln!(string, "Bw: {:?}", channel_cfg.bandwidth).unwrap();
        writeln!(string, "RSSI: {} dBm", rssi).unwrap();

        self.display.clear(BinaryColor::Off).unwrap();

        let font = MonoTextStyle::new(&FONT_8X13_BOLD, BinaryColor::On);
        Text::new(string.as_str(), Point::new(1, 13), font)
            .draw(&mut self.display)
            .unwrap();

        self.display.flush().unwrap();
    }
}
