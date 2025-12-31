#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use rquangsheng as _; // global logger + panicking-behavior + memory layout

use rtic_monotonics::systick::prelude::*;

systick_monotonic!(Mono, 1_00);

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    device = rquangsheng::dp30g030_hal,
    // TODO: Replace the `FreeInterrupt1, ...` with free interrupt vectors if software tasks are used
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [IWDT]
)]

mod app {
    use embedded_hal::digital::OutputPin;
    use rquangsheng::dp30g030_hal::gpio::{Output, Pin, Port};
    use rquangsheng::dp30g030_hal::uart;
    use rtic_monotonics::{fugit::ExtU32, Monotonic as _};

    use crate::Mono;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        pin_flashlight: Pin<Output>,
        pin_backlight: Pin<Output>,
        uart1: uart::Uart1,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        // TODO setup monotonic if used
        // let sysclk = { /* clock setup + returning sysclk as an u32 */ };
        // let token = rtic_monotonics::create_systick_token!();
        // rtic_monotonics::systick::Systick::new(cx.core.SYST, sysclk, token);

        let pin_flashlight =
            Pin::new(Port::C, 3).into_push_pull_output(&cx.device.SYSCON, &cx.device.PORTCON);
        let pin_backlight =
            Pin::new(Port::B, 6).into_push_pull_output(&cx.device.SYSCON, &cx.device.PORTCON);

        // UART example: UART1 on PA7 (TX) / PA8 (RX), 38400-8N1.
        let uart1_tx =
            uart::TxPin::<rquangsheng::dp30g030_hal::UART1>::new(Pin::new(Port::A, 7)).unwrap();
        let uart1_rx =
            uart::RxPin::<rquangsheng::dp30g030_hal::UART1>::new(Pin::new(Port::A, 8)).unwrap();
        let uart1_cfg = uart::Config::new(48_000_000, 38_400);
        let uart1: uart::Uart1 = uart::Uart::<
            rquangsheng::dp30g030_hal::UART1,
            uart::TxPin<rquangsheng::dp30g030_hal::UART1>,
            uart::RxPin<rquangsheng::dp30g030_hal::UART1>,
        >::new(
            cx.device.UART1,
            &cx.device.SYSCON,
            &cx.device.PORTCON,
            uart1_tx,
            uart1_rx,
            uart1_cfg,
        )
        .unwrap();

        Mono::start(cx.core.SYST, 48_000_000);

        task1::spawn().ok();
        uart_task::spawn().ok();

        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                // Initialization of local resources go here
                pin_flashlight,
                pin_backlight,
                uart1,
            },
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }

    // TODO: Add tasks
    #[task(priority = 1, local = [pin_flashlight, pin_backlight])]
    async fn task1(cx: task1::Context<Local>) {
        defmt::info!("Hello from task1!");

        loop {
            cx.local.pin_flashlight.set_high();
            cx.local.pin_backlight.set_low();
            Mono::delay(500.millis()).await;

            cx.local.pin_flashlight.set_low();
            cx.local.pin_backlight.set_high();
            Mono::delay(500.millis()).await;
        }
    }

    // UART demo task: writes a message periodically on UART1.
    #[task(priority = 1, local = [uart1])]
    async fn uart_task(cx: uart_task::Context) {
        use embedded_hal_nb::serial::Write as _;
        use nb::block;

        loop {
            for &b in b"Hello from UART1!\r\n" {
                let _ = block!(cx.local.uart1.write(b));
            }
            let _ = block!(cx.local.uart1.flush());

            Mono::delay(1.secs()).await;
        }
    }
}
