#![no_main]
#![no_std]

// Heavy inspiration from TeXitoi's keyseebee firmware
// https://github.com/TeXitoi/keyseebee/tree/keyberon-master

use panic_halt as _;

use core::convert::Infallible;

use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::Rectangle,
    style::{PrimitiveStyle, PrimitiveStyleBuilder},
};
use embedded_hal::digital::v2::{InputPin, OutputPin};
use hal::gpio::{gpioa, gpiob, gpioc, gpiod, gpioe, gpiof, Input, Output, PullUp, PushPull};
use hal::i2c::I2c;
use hal::prelude::*;
use hal::usb;
use hal::{stm32, timers};
use heapless::consts::*;
use keyberon::action::{k, l, Action, Action::Trans, HoldTapConfig};
use keyberon::debounce::Debouncer;
use keyberon::impl_heterogenous_array;
use keyberon::key_code::{KbHidReport, KeyCode::*};
use keyberon::layout::Layout;
use keyberon::matrix::{Matrix, PressedKeys};
use rotary_encoder_hal::{Direction, Rotary};
use rtic::app;
use ssd1306::{prelude::*, Builder, I2CDIBuilder};
use stm32f0xx_hal as hal;
use core::sync::atomic::{AtomicBool, Ordering};
use usb_device::bus::UsbBusAllocator;
use usb_device::class::UsbClass as _;
type UsbClass = keyberon::Class<'static, usb::UsbBusType, CapsLock>;
type UsbDevice = usb_device::device::UsbDevice<'static, usb::UsbBusType>;
type Display = ssd1306::mode::GraphicsMode<
    ssd1306::prelude::I2CInterface<
        stm32f0xx_hal::i2c::I2c<
            stm32::I2C1,
            stm32f0xx_hal::gpio::gpiob::PB6<
                stm32f0xx_hal::gpio::Alternate<stm32f0xx_hal::gpio::AF1>,
            >,
            stm32f0xx_hal::gpio::gpiob::PB7<
                stm32f0xx_hal::gpio::Alternate<stm32f0xx_hal::gpio::AF1>,
            >,
        >,
    >,
    ssd1306::displaysize::DisplaySize128x32,
>;

// This count divided by the OLED update rate is the period
const OLED_TIMEOUT_COUNT: u32 = 300;

const SHIFT_UP: Action<()> = Action::HoldTap {
    timeout: 200,
    config: HoldTapConfig::HoldOnOtherKeyPress,
    tap_hold_interval: 0,
    hold: &k(RShift),
    tap: &k(Up),
};
const CTRL_RIGHT: Action<()> = Action::HoldTap {
    timeout: 200,
    config: HoldTapConfig::HoldOnOtherKeyPress,
    tap_hold_interval: 0,
    hold: &k(RCtrl),
    tap: &k(Right),
};
const GUI_DOWN: Action<()> = Action::HoldTap {
    timeout: 200,
    config: HoldTapConfig::HoldOnOtherKeyPress,
    tap_hold_interval: 0,
    hold: &k(RGui),
    tap: &k(Down),
};
const ALT_LEFT: Action<()> = Action::HoldTap {
    timeout: 200,
    config: HoldTapConfig::HoldOnOtherKeyPress,
    tap_hold_interval: 0,
    hold: &k(RAlt),
    tap: &k(Left),
};

// Giant Column struct allows for all direct pins in a "flat" array
// See Grabert's hardware repo for pinout
#[rustfmt::skip]
pub struct Cols(
    gpiod::PD2<Input<PullUp>>, gpiod::PD4<Input<PullUp>>, gpiob::PB3<Input<PullUp>>,
    gpioe::PE0<Input<PullUp>>, gpioe::PE5<Input<PullUp>>, gpioc::PC0<Input<PullUp>>,
    gpiof::PF2<Input<PullUp>>, gpioa::PA2<Input<PullUp>>, gpiob::PB0<Input<PullUp>>,
    gpioe::PE9<Input<PullUp>>, gpioe::PE14<Input<PullUp>>, gpiob::PB14<Input<PullUp>>,
    gpiod::PD10<Input<PullUp>>, gpioc::PC6<Input<PullUp>>, gpioc::PC7<Input<PullUp>>,
    gpiod::PD3<Input<PullUp>>, gpiod::PD7<Input<PullUp>>, gpiob::PB9<Input<PullUp>>,
    gpioe::PE4<Input<PullUp>>, gpiof::PF10<Input<PullUp>>, gpioc::PC3<Input<PullUp>>,
    gpioa::PA1<Input<PullUp>>, gpioa::PA7<Input<PullUp>>, gpiob::PB1<Input<PullUp>>,
    gpioe::PE10<Input<PullUp>>, gpioe::PE15<Input<PullUp>>, gpiob::PB15<Input<PullUp>>,
    gpiod::PD11<Input<PullUp>>, gpiod::PD15<Input<PullUp>>, gpiod::PD5<Input<PullUp>>,
    gpiob::PB8<Input<PullUp>>, gpioe::PE3<Input<PullUp>>, gpiof::PF9<Input<PullUp>>,
    gpioc::PC2<Input<PullUp>>, gpioa::PA0<Input<PullUp>>, gpioa::PA3<Input<PullUp>>,
    gpioc::PC4<Input<PullUp>>, gpiob::PB2<Input<PullUp>>, gpioe::PE11<Input<PullUp>>,
    gpiob::PB10<Input<PullUp>>, gpiod::PD8<Input<PullUp>>, gpiod::PD12<Input<PullUp>>,
    gpiod::PD6<Input<PullUp>>, gpiob::PB5<Input<PullUp>>, gpioe::PE2<Input<PullUp>>,
    gpioc::PC13<Input<PullUp>>, gpioc::PC1<Input<PullUp>>, gpiof::PF3<Input<PullUp>>,
    gpioa::PA4<Input<PullUp>>, gpioa::PA6<Input<PullUp>>, gpioc::PC5<Input<PullUp>>,
    gpioe::PE7<Input<PullUp>>, gpioe::PE12<Input<PullUp>>, gpiob::PB11<Input<PullUp>>,
    gpiod::PD9<Input<PullUp>>, gpiod::PD13<Input<PullUp>>, gpiob::PB4<Input<PullUp>>,
    gpioe::PE1<Input<PullUp>>, gpioe::PE6<Input<PullUp>>, gpioa::PA5<Input<PullUp>>,
    gpioe::PE8<Input<PullUp>>, gpioe::PE13<Input<PullUp>>, gpiob::PB12<Input<PullUp>>,
    gpiob::PB13<Input<PullUp>>, gpiod::PD14<Input<PullUp>>, gpioc::PC8<Input<PullUp>>,
);
impl_heterogenous_array! {
    Cols,
    dyn InputPin<Error = Infallible>,
    U66,
    [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
    30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43,
    44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57,
    58, 59, 60, 61, 62, 63, 64, 65]
}

// Unused pin for "pseudo row"
pub struct Rows(gpioc::PC11<Output<PushPull>>);
impl_heterogenous_array! {
    Rows,
    dyn OutputPin<Error = Infallible>,
    U1,
    [0]
}

static CAPS_LOCK: AtomicBool = AtomicBool::new(false);
pub struct CapsLock {}
impl keyberon::keyboard::Leds for CapsLock {
    fn caps_lock(&mut self, status: bool) {
        CAPS_LOCK.store(status, Ordering::Relaxed);
    }
}

// TODO: Develop a better layout
#[rustfmt::skip]
pub static LAYERS: keyberon::layout::Layers<()> = &[
    &[&[
        k(Grave), k(Kb1), k(Kb2), k(Kb3), k(Kb4), k(Kb5), k(Kb6), k(Kb7), k(Kb8), k(Kb9), k(Kb0), k(Minus), k(Equal), k(BSpace), k(BSpace),
        k(Tab), k(Q), k(W), k(E), k(R), k(T), k(Y), k(U), k(I), k(O), k(P), k(LBracket), k(RBracket), k(Bslash),
        k(CapsLock), k(A), k(S), k(D), k(F), k(G), k(H), k(J), k(K), k(L), k(SColon), k(Quote), k(Enter),
        k(LShift), k(LShift), k(Z), k(X), k(C), k(V), k(B), k(N), k(M), k(Comma), k(Dot), k(Slash), SHIFT_UP, k(Up),
        k(LCtrl), k(LGui), k(LAlt), k(Space), l(1), ALT_LEFT, ALT_LEFT, GUI_DOWN, CTRL_RIGHT, k(MediaMute), k(MediaVolDown), k(MediaVolUp),
    ]],
    &[&[
        k(Escape), k(F1), k(F2), k(F3), k(F4), k(F5), k(F6), k(F7), k(F8), k(F9), k(F10), k(F11), k(F12), k(Delete), k(Delete),
        Trans, k(MediaSleep), Trans, k(Up), Trans, Trans, Trans, Trans, k(Insert), Trans, k(PScreen), k(Home), k(End), Trans,
        Trans, Trans, k(Left), k(Down), k(Right), Trans, Trans, k(MediaPlayPause), k(MediaNextSong), Trans, k(MediaVolDown), k(MediaVolUp), Trans,
        Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, k(Up), k(Up),
        Trans, l(2), Trans, Trans, Trans, k(Left), k(Left), k(Down), k(Right), Trans, Trans, Trans
    ]],
    &[&[
        Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans,
        Trans, Action::Custom(()), Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans,
        Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans,
        Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans,
        Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans, Trans
    ]],
];

#[app(device = crate::hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_dev: UsbDevice,
        usb_class: UsbClass,
        scan_timer: timers::Timer<stm32::TIM3>,
        oled_timer: timers::Timer<stm32::TIM2>,
        iwdg: hal::watchdog::Watchdog,
        display: Display,
        filled_rect_style: PrimitiveStyle<BinaryColor>,
        stroke_rect_style: PrimitiveStyle<BinaryColor>,
        matrix: Matrix<Cols, Rows>,
        layout: Layout<()>,
        debouncer: Debouncer<PressedKeys<U1, U66>>,
        oled_timeout_count: u32,
        encoder: Rotary<gpioc::PC9<Input<PullUp>>, gpioa::PA8<Input<PullUp>>>,
    }

    #[init]
    fn init(mut c: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<UsbBusAllocator<usb::UsbBusType>> = None;

        let mut rcc = c
            .device
            .RCC
            .configure()
            .hsi48()
            .enable_crs(c.device.CRS)
            .sysclk(48.mhz())
            .pclk(24.mhz())
            .freeze(&mut c.device.FLASH);

        let mut iwdg = hal::watchdog::Watchdog::new(c.device.IWDG);
        iwdg.start(1.hz());

        // Splitting each Port at once causes power issues
        // IWDG takes care of any power issues with a reset
        // after power has stabilized, while keeping enumeration times short
        let gpioa = c.device.GPIOA.split(&mut rcc);
        let gpiob = c.device.GPIOB.split(&mut rcc);
        let gpioc = c.device.GPIOC.split(&mut rcc);
        let gpiod = c.device.GPIOD.split(&mut rcc);
        let gpioe = c.device.GPIOE.split(&mut rcc);
        let gpiof = c.device.GPIOF.split(&mut rcc);

        // Setup I2C and OLED Display
        let pb6 = gpiob.pb6;
        let pb7 = gpiob.pb7;
        let (scl, sda) = cortex_m::interrupt::free(move |cs| {
            (pb6.into_alternate_af1(cs), pb7.into_alternate_af1(cs))
        });

        let i2c = I2c::i2c1(c.device.I2C1, (scl, sda), 400.khz(), &mut rcc);
        let i2c_interface = I2CDIBuilder::new().init(i2c);
        let mut disp: GraphicsMode<_, _> = Builder::new()
            .size(DisplaySize128x32)
            .connect(i2c_interface)
            .into();
        disp.init().unwrap();

        let pc9 = gpioc.pc9;
        let pa8 = gpioa.pa8;
        let (pina, pinb) = cortex_m::interrupt::free(move |cs| {
            (pc9.into_pull_up_input(cs), pa8.into_pull_up_input(cs))
        });
        let encoder = Rotary::new(pina, pinb);

        let filled_rect_style = PrimitiveStyleBuilder::new()
            .fill_color(BinaryColor::On)
            .build();
        let stroke_rect_style = PrimitiveStyleBuilder::new()
            .stroke_color(BinaryColor::On)
            .stroke_width(1)
            .build();

        // TODO: Initialize OLED with some default image
        disp.clear();
        disp.flush().unwrap();

        let caps_lock = CapsLock {};
        let usb = usb::Peripheral {
            usb: c.device.USB,
            pin_dm: gpioa.pa11,
            pin_dp: gpioa.pa12,
        };
        *USB_BUS = Some(usb::UsbBusType::new(usb));
        let usb_bus = USB_BUS.as_ref().unwrap();
        let usb_class = keyberon::new_class(usb_bus, caps_lock);
        let usb_dev = keyberon::new_device(usb_bus);

        // Prevent port move
        #[rustfmt::skip]
        let (
            pd2, pd4, pb3, pe0, pe5, pc0, pf2, pa2, pb0, pe9,
            pe14, pb14, pd10, pc6, pc7, pd3, pd7, pb9, pe4, pf10,
            pc3, pa1, pa7, pb1, pe10, pe15, pb15, pd11, pd15, pd5,
            pb8, pe3, pf9, pc2, pa0, pa3, pc4, pb2, pe11, pb10,
            pd8, pd12, pd6, pb5, pe2, pc13, pc1, pf3, pa4, pa6,
            pc5, pe7, pe12, pb11, pd9, pd13, pb4, pe1, pe6, pa5,
            pe8, pe13, pb12, pb13, pd14, pc8
            ) = (
            gpiod.pd2, gpiod.pd4, gpiob.pb3, gpioe.pe0, gpioe.pe5,
            gpioc.pc0, gpiof.pf2, gpioa.pa2, gpiob.pb0, gpioe.pe9,
            gpioe.pe14, gpiob.pb14, gpiod.pd10, gpioc.pc6, gpioc.pc7,
            gpiod.pd3, gpiod.pd7, gpiob.pb9, gpioe.pe4, gpiof.pf10,
            gpioc.pc3, gpioa.pa1, gpioa.pa7, gpiob.pb1, gpioe.pe10,
            gpioe.pe15, gpiob.pb15, gpiod.pd11, gpiod.pd15, gpiod.pd5,
            gpiob.pb8, gpioe.pe3, gpiof.pf9, gpioc.pc2, gpioa.pa0,
            gpioa.pa3, gpioc.pc4, gpiob.pb2, gpioe.pe11, gpiob.pb10,
            gpiod.pd8, gpiod.pd12, gpiod.pd6, gpiob.pb5, gpioe.pe2,
            gpioc.pc13, gpioc.pc1, gpiof.pf3, gpioa.pa4, gpioa.pa6,
            gpioc.pc5, gpioe.pe7, gpioe.pe12, gpiob.pb11, gpiod.pd9,
            gpiod.pd13, gpiob.pb4, gpioe.pe1, gpioe.pe6, gpioa.pa5,
            gpioe.pe8, gpioe.pe13, gpiob.pb12, gpiob.pb13, gpiod.pd14,
            gpioc.pc8
            );

        let pc11 = gpioc.pc11;
        #[rustfmt::skip]
        let matrix = cortex_m::interrupt::free(move |cs| {
            Matrix::new(
                Cols(
                    pd2.into_pull_up_input(cs), pd4.into_pull_up_input(cs), pb3.into_pull_up_input(cs),
                    pe0.into_pull_up_input(cs), pe5.into_pull_up_input(cs), pc0.into_pull_up_input(cs),
                    pf2.into_pull_up_input(cs), pa2.into_pull_up_input(cs), pb0.into_pull_up_input(cs),
                    pe9.into_pull_up_input(cs), pe14.into_pull_up_input(cs), pb14.into_pull_up_input(cs),
                    pd10.into_pull_up_input(cs), pc6.into_pull_up_input(cs), pc7.into_pull_up_input(cs),
                    pd3.into_pull_up_input(cs), pd7.into_pull_up_input(cs), pb9.into_pull_up_input(cs),
                    pe4.into_pull_up_input(cs), pf10.into_pull_up_input(cs), pc3.into_pull_up_input(cs),
                    pa1.into_pull_up_input(cs), pa7.into_pull_up_input(cs), pb1.into_pull_up_input(cs),
                    pe10.into_pull_up_input(cs), pe15.into_pull_up_input(cs), pb15.into_pull_up_input(cs),
                    pd11.into_pull_up_input(cs), pd15.into_pull_up_input(cs), pd5.into_pull_up_input(cs),
                    pb8.into_pull_up_input(cs), pe3.into_pull_up_input(cs), pf9.into_pull_up_input(cs),
                    pc2.into_pull_up_input(cs), pa0.into_pull_up_input(cs), pa3.into_pull_up_input(cs),
                    pc4.into_pull_up_input(cs), pb2.into_pull_up_input(cs), pe11.into_pull_up_input(cs),
                    pb10.into_pull_up_input(cs), pd8.into_pull_up_input(cs), pd12.into_pull_up_input(cs),
                    pd6.into_pull_up_input(cs), pb5.into_pull_up_input(cs), pe2.into_pull_up_input(cs),
                    pc13.into_pull_up_input(cs), pc1.into_pull_up_input(cs), pf3.into_pull_up_input(cs),
                    pa4.into_pull_up_input(cs), pa6.into_pull_up_input(cs), pc5.into_pull_up_input(cs),
                    pe7.into_pull_up_input(cs), pe12.into_pull_up_input(cs), pb11.into_pull_up_input(cs),
                    pd9.into_pull_up_input(cs), pd13.into_pull_up_input(cs), pb4.into_pull_up_input(cs),
                    pe1.into_pull_up_input(cs), pe6.into_pull_up_input(cs), pa5.into_pull_up_input(cs),
                    pe8.into_pull_up_input(cs), pe13.into_pull_up_input(cs), pb12.into_pull_up_input(cs),
                    pb13.into_pull_up_input(cs), pd14.into_pull_up_input(cs), pc8.into_pull_up_input(cs),
                ),
                Rows(pc11.into_push_pull_output(cs)),
            )
        });

        let mut scan_timer = timers::Timer::tim3(c.device.TIM3, 1000.hz(), &mut rcc);
        scan_timer.listen(timers::Event::TimeOut);

        // TODO: Determine best OLED refresh rate
        let mut oled_timer = timers::Timer::tim2(c.device.TIM2, 10.hz(), &mut rcc);
        oled_timer.listen(timers::Event::TimeOut);

        init::LateResources {
            usb_dev,
            usb_class,
            scan_timer,
            oled_timer,
            iwdg,
            display: disp,
            filled_rect_style,
            stroke_rect_style,
            matrix: matrix.unwrap(),
            layout: Layout::new(LAYERS),
            debouncer: Debouncer::new(PressedKeys::default(), PressedKeys::default(), 5),
            oled_timeout_count: 0,
            encoder,
        }
    }

    #[task(binds = USB, priority = 4, resources = [usb_dev, usb_class, oled_timeout_count])]
    fn usb_rx(c: usb_rx::Context) {
        // USB activity resets OLED counter to keep the screen on
        *c.resources.oled_timeout_count = 0;
        if c.resources.usb_dev.poll(&mut [c.resources.usb_class]) {
            c.resources.usb_class.poll();
        }
    }

    // Code from https://github.com/TeXitoi/keyberon-f4/blob/7c1d84bd9d361f77703128603158beab8856b3a7/src/main.rs#L230
    #[task(
        binds = TIM3,
        priority = 2,
        resources = [scan_timer, matrix, debouncer, layout, encoder, usb_dev, usb_class],
    )]
    fn scan_timer_irq(mut c: scan_timer_irq::Context) {
        static mut RESOLUTION_COUNT: u8 = 0;
        c.resources.scan_timer.wait().ok();

        // Read the encoder pins through an update()
        // Then if 4 increments have occured (one physical click),
        // press and release the "buttons" in the layout
        match c.resources.encoder.update().unwrap() {
            Direction::Clockwise => {
                if *RESOLUTION_COUNT == 3 {
                    c.resources
                        .layout
                        .event(keyberon::layout::Event::Press(0, 66));
                    c.resources
                        .layout
                        .event(keyberon::layout::Event::Release(0, 66));
                    *RESOLUTION_COUNT = 0;
                } else {
                    *RESOLUTION_COUNT += 1;
                }
            }
            Direction::CounterClockwise => {
                if *RESOLUTION_COUNT == 3 {
                    c.resources
                        .layout
                        .event(keyberon::layout::Event::Press(0, 67));
                    c.resources
                        .layout
                        .event(keyberon::layout::Event::Release(0, 67));
                    *RESOLUTION_COUNT = 0;
                } else {
                    *RESOLUTION_COUNT += 1;
                }
            }
            Direction::None => {}
        }

        for event in c
            .resources
            .debouncer
            .events(c.resources.matrix.get().unwrap())
        {
            c.resources.layout.event(event);
        }
        match c.resources.layout.tick() {
            keyberon::layout::CustomEvent::Release(()) => unsafe {
                // Enter DFU
                cortex_m::asm::bootload(0x1FFFC800 as *const u32);
            },
            _ => (),
        }
        send_report(c.resources.layout.keycodes(), &mut c.resources.usb_class);
    }

    // Low Priority OLED update Task
    // TODO: Determine if i2c calls are blocking
    #[task(
        binds = TIM2,
        priority = 1,
        resources = [iwdg, oled_timer, display, &filled_rect_style, &stroke_rect_style, oled_timeout_count, debouncer],
    )]
    fn oled_timer_irq(mut c: oled_timer_irq::Context) {
        static mut OLED_ON: bool = false;
        static mut PRESSED: [bool; 66] = [false; 66];
        // TODO make the keymap image generated in a const fn
        static mut SQUARES: [Rectangle; 66] = [
            Rectangle::new(Point::new(4, 2), Point::new(11, 7)),
            Rectangle::new(Point::new(11, 2), Point::new(18, 7)),
            Rectangle::new(Point::new(18, 2), Point::new(25, 7)),
            Rectangle::new(Point::new(25, 2), Point::new(32, 7)),
            Rectangle::new(Point::new(32, 2), Point::new(39, 7)),
            Rectangle::new(Point::new(39, 2), Point::new(46, 7)),
            Rectangle::new(Point::new(46, 2), Point::new(53, 7)),
            Rectangle::new(Point::new(53, 2), Point::new(60, 7)),
            Rectangle::new(Point::new(60, 2), Point::new(67, 7)),
            Rectangle::new(Point::new(67, 2), Point::new(74, 7)),
            Rectangle::new(Point::new(74, 2), Point::new(81, 7)),
            Rectangle::new(Point::new(81, 2), Point::new(88, 7)),
            Rectangle::new(Point::new(88, 2), Point::new(95, 7)),
            Rectangle::new(Point::new(95, 2), Point::new(102, 7)),
            Rectangle::new(Point::new(102, 2), Point::new(109, 7)),
            Rectangle::new(Point::new(4, 7), Point::new(15, 13)),
            Rectangle::new(Point::new(15, 7), Point::new(22, 13)),
            Rectangle::new(Point::new(22, 7), Point::new(29, 13)),
            Rectangle::new(Point::new(29, 7), Point::new(36, 13)),
            Rectangle::new(Point::new(36, 7), Point::new(43, 13)),
            Rectangle::new(Point::new(43, 7), Point::new(50, 13)),
            Rectangle::new(Point::new(50, 7), Point::new(57, 13)),
            Rectangle::new(Point::new(57, 7), Point::new(64, 13)),
            Rectangle::new(Point::new(64, 7), Point::new(71, 13)),
            Rectangle::new(Point::new(71, 7), Point::new(78, 13)),
            Rectangle::new(Point::new(78, 7), Point::new(85, 13)),
            Rectangle::new(Point::new(85, 7), Point::new(92, 13)),
            Rectangle::new(Point::new(92, 7), Point::new(99, 13)),
            Rectangle::new(Point::new(99, 7), Point::new(109, 13)),
            Rectangle::new(Point::new(4, 13), Point::new(17, 19)),
            Rectangle::new(Point::new(17, 13), Point::new(24, 19)),
            Rectangle::new(Point::new(24, 13), Point::new(31, 19)),
            Rectangle::new(Point::new(31, 13), Point::new(38, 19)),
            Rectangle::new(Point::new(38, 13), Point::new(45, 19)),
            Rectangle::new(Point::new(45, 13), Point::new(52, 19)),
            Rectangle::new(Point::new(52, 13), Point::new(59, 19)),
            Rectangle::new(Point::new(59, 13), Point::new(66, 19)),
            Rectangle::new(Point::new(66, 13), Point::new(73, 19)),
            Rectangle::new(Point::new(73, 13), Point::new(80, 19)),
            Rectangle::new(Point::new(80, 13), Point::new(87, 19)),
            Rectangle::new(Point::new(87, 13), Point::new(94, 19)),
            Rectangle::new(Point::new(94, 13), Point::new(109, 19)),
            Rectangle::new(Point::new(4, 19), Point::new(13, 25)),
            Rectangle::new(Point::new(13, 19), Point::new(20, 25)),
            Rectangle::new(Point::new(20, 19), Point::new(27, 25)),
            Rectangle::new(Point::new(27, 19), Point::new(34, 25)),
            Rectangle::new(Point::new(34, 19), Point::new(41, 25)),
            Rectangle::new(Point::new(41, 19), Point::new(48, 25)),
            Rectangle::new(Point::new(48, 19), Point::new(55, 25)),
            Rectangle::new(Point::new(55, 19), Point::new(62, 25)),
            Rectangle::new(Point::new(62, 19), Point::new(69, 25)),
            Rectangle::new(Point::new(69, 19), Point::new(76, 25)),
            Rectangle::new(Point::new(76, 19), Point::new(83, 25)),
            Rectangle::new(Point::new(83, 19), Point::new(90, 25)),
            Rectangle::new(Point::new(90, 19), Point::new(102, 25)),
            Rectangle::new(Point::new(102, 19), Point::new(109, 25)),
            Rectangle::new(Point::new(4, 25), Point::new(13, 31)),
            Rectangle::new(Point::new(13, 25), Point::new(22, 31)),
            Rectangle::new(Point::new(22, 25), Point::new(31, 31)),
            Rectangle::new(Point::new(31, 25), Point::new(74, 31)),
            Rectangle::new(Point::new(74, 25), Point::new(81, 31)),
            Rectangle::new(Point::new(81, 25), Point::new(88, 31)),
            Rectangle::new(Point::new(88, 25), Point::new(95, 31)),
            Rectangle::new(Point::new(95, 25), Point::new(102, 31)),
            Rectangle::new(Point::new(102, 25), Point::new(109, 31)),
            Rectangle::new(Point::new(117, 2), Point::new(122, 7)),
        ];

        c.resources.iwdg.feed();

        c.resources.oled_timeout_count.lock(|oled_timeout_count| {
            *oled_timeout_count += 1;
            if *oled_timeout_count > OLED_TIMEOUT_COUNT {
                *OLED_ON = false;
            } else {
                *OLED_ON = true;
            }
        });
        c.resources.display.clear();
        if *OLED_ON {
            *PRESSED = [false; 66];
            c.resources.debouncer.lock(|debaser| {
                for switch in debaser.get().iter_pressed() {
                    PRESSED[switch.1] = true;
                }
            });
            for (idx, is_pressed) in PRESSED.iter().enumerate() {
                if *is_pressed {
                    SQUARES[idx]
                        .into_styled(*c.resources.filled_rect_style)
                        .draw(c.resources.display)
                        .unwrap();
                } else {
                    SQUARES[idx]
                        .into_styled(*c.resources.stroke_rect_style)
                        .draw(c.resources.display)
                        .unwrap();
                }
            }
            if CAPS_LOCK.load(Ordering::Relaxed) {
                SQUARES[29]
                    .into_styled(*c.resources.filled_rect_style)
                    .draw(c.resources.display)
                    .unwrap();
            }
        }
        c.resources.display.flush().unwrap();
    }

    // Interrupts for rtic "software" tasks
    extern "C" {
        fn CEC_CAN();
    }
};

fn send_report(
    iter: impl Iterator<Item = keyberon::key_code::KeyCode>,
    usb_class: &mut resources::usb_class<'_>,
) {
    use rtic::Mutex;
    let report: KbHidReport = iter.collect();
    if usb_class.lock(|k| k.device_mut().set_keyboard_report(report.clone())) {
        while let Ok(0) = usb_class.lock(|k| k.write(report.as_bytes())) {}
    }
}
