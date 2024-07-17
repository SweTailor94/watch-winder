// This is the program for Watch Winder

#![no_std]
#![no_main]

mod ds1302;
mod ds1302_pio_spi;
mod format_buffer;

use defmt::{info,error};
use ds1302::Ds1302;
use ds1302_pio_spi::PioSpiDs1302;
use embassy_executor::Spawner;
use embassy_futures::select::{self, Either};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio;
use embassy_rp::peripherals::PIO0;
use embassy_rp::peripherals::{
    I2C1, PIN_16, PIN_18, PIN_19, PIN_20, PIN_21, PIN_6, PIN_7, PIN_8, PIN_9,
};
use embassy_rp::pio::Pio;
use embassy_sync::pubsub::PubSubChannel;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Timer};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use gpio::{Input, Level, Output, Pull};
use {defmt_rtt as _, panic_probe as _};
// For Motor
use uln2003::{StepperMotor, ULN2003};

// For screen, ssd1306
use core::fmt::Write;
use embassy_rp::i2c::{self, Config};
use embassy_sync::channel::Channel;
use embedded_graphics::{
    image::Image,
    mono_font::{ascii::FONT_7X13, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use format_buffer::FmtBuf;
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::prelude::{DisplayConfig, I2CInterface};
use ssd1306::rotation::DisplayRotation;
use ssd1306::size::DisplaySize128x64;
use ssd1306::{I2CDisplayInterface, Ssd1306};
use tinybmp::Bmp;
bind_interrupts!(struct Irqs {
    I2C1_IRQ => embassy_rp::i2c::InterruptHandler<I2C1>;
});
// For RTC
bind_interrupts!(struct PioIrqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});

type MotorControl = ULN2003<
    Output<'static, PIN_6>,
    Output<'static, PIN_7>,
    Output<'static, PIN_8>,
    Output<'static, PIN_9>,
    Delay,
>;

const START_HOUR: u8 = 8;
const START_MINUTE: u8 = 0;
const PAUSE_MINUTES: u8 = 1;

static KEY_CHANNEL: Channel<CriticalSectionRawMutex, Key, 2> = Channel::new();
static RTC_COMMAND: Channel<CriticalSectionRawMutex, RtcCommand, 2> = Channel::new();
static ACTION: Channel<CriticalSectionRawMutex, Action, 10> = Channel::new();
static SETTINGS_COMMAND: Channel<CriticalSectionRawMutex, FlashCmd, 2> = Channel::new();
static MOTOR_CMD_CHANNEL: Channel<CriticalSectionRawMutex, MotorCommand, 1> = Channel::new();

static SETTINGS_PUBLISHER: PubSubChannel<CriticalSectionRawMutex, Settings, 1, 2, 1> =
    PubSubChannel::new();
static RTC_PUBLISHER: PubSubChannel<CriticalSectionRawMutex, WwTime, 1, 3, 1> =
    PubSubChannel::new();

static TURN_COUNT: Signal<CriticalSectionRawMutex, (i32, u16)> = Signal::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let led = Output::new(p.PIN_16, Level::Low);

    // Screen
    // Pins for I2C to the Screen
    let sda = p.PIN_14;
    let scl = p.PIN_15;
    info!("set up i2c ");
    let mut i2c_config = Config::default();
    i2c_config.frequency = 1_000_000;
    let i2c = i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, i2c_config);
    let interface = I2CDisplayInterface::new(i2c);
    let display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    // Buttons
    let button_up = Input::new(p.PIN_18, Pull::Up);
    let button_down = Input::new(p.PIN_19, Pull::Up);
    let button_enter = Input::new(p.PIN_20, Pull::Up);
    let button_back = Input::new(p.PIN_21, Pull::Up);
    spawner
        .spawn(run_key_task(
            button_up,
            button_down,
            button_enter,
            button_back,
        ))
        .unwrap();

    // Show menu task, sends multiple lines to displaytask deppending on app state. No peripherals
    spawner.spawn(run_show_menu_task(display)).unwrap();

  
    // Motor control task
    let in1 = Output::new(p.PIN_6, Level::Low);
    let in2 = Output::new(p.PIN_7, Level::Low);
    let in3 = Output::new(p.PIN_8, Level::Low);
    let in4 = Output::new(p.PIN_9, Level::Low);
    spawner
        .spawn(run_motor_control(led, in1, in2, in3, in4))
        .unwrap();

    let mut sub_time: embassy_sync::pubsub::Subscriber<CriticalSectionRawMutex, WwTime, 1, 3, 1> = RTC_PUBLISHER.subscriber().unwrap();
    // RTC and persistent memory
    let cs = Output::new(p.PIN_12, Level::Low);
    let mut pio = Pio::new(p.PIO0, PioIrqs);
    let spi: PioSpiDs1302<
        embassy_rp::peripherals::PIN_12,
        PIO0,
        0,
    > = PioSpiDs1302::new(
        &mut pio.common,
        pio.sm0,
        pio.irq0,
        cs,
        p.PIN_11,
        p.PIN_10,
    );

    spawner.spawn(run_rtc_and_persistence_task(spi)).unwrap();

    // the main app state machine. Maybe should be in main loop?
    // spawner.spawn(run_app_state_task()).unwrap();

    SETTINGS_COMMAND.send(FlashCmd::SendSettings).await;
    let mut settings_sub = SETTINGS_PUBLISHER.subscriber().unwrap();
    let settings = settings_sub.next_message_pure().await;
    let time = sub_time.next_message_pure().await;
    let mut states = AppStateHandler::new(&settings, time);
    // Lets wait 1 seconds to show the splash screen
    Timer::after(Duration::from_secs(1)).await;
    ACTION.send(Action::ShowMenu(states.get_state())).await;
    let mut show_menu;
    loop {
        match select::select(KEY_CHANNEL.receive() ,sub_time.next_message_pure()).await{
            Either::First(key) => {
                show_menu = true;
                let action = states.key_pressed(key);
                match action {
                    Action::DoNothing => {
                        show_menu = false;
                    }
                    Action::SetTime { hour,min } => {
                        RTC_COMMAND.send(RtcCommand::SetNewTime(hour,min)).await;
                    }
                    Action::StopTurning => {
                        MOTOR_CMD_CHANNEL.send(MotorCommand::Stop).await;
                        MOTOR_CMD_CHANNEL.send(MotorCommand::Park).await;
                    }
                    Action::StartTurning => {
                        MOTOR_CMD_CHANNEL.send(MotorCommand::Start).await;
                    }
                    Action::StepMotor(d) => {
                        MOTOR_CMD_CHANNEL.send(MotorCommand::Step(d)).await;
                        show_menu = false;
                    }
                    Action::ShowMenu(s) => {
                        ACTION.send(Action::ShowMenu(s)).await;
                        show_menu = false;
                    }
                    Action::UpdateSettings(settings) => {
                        SETTINGS_COMMAND.send(FlashCmd::SetSettings(settings)).await;
                    }
                    Action::HourUp => {
                        RTC_COMMAND.send(RtcCommand::HourUp).await;
                    }
                    Action::HourDown => {
                        RTC_COMMAND.send(RtcCommand::HourDown).await;
                    }
                    Action::MinuteUp => {
                        RTC_COMMAND.send(RtcCommand::MinuteUp).await;
                    }
                    Action::MinuteDown => {
                        RTC_COMMAND.send(RtcCommand::MinuteDown).await;
                    }
                }
                if show_menu {
                    ACTION.send(Action::ShowMenu(states.get_state())).await;
                }
                if let Some(s) = settings_sub.try_next_message_pure() {
                    states.settings = s;
                }
            }
            Either::Second(t) => {
                states.set_time(t);
            }
        }    
    }
}


#[embassy_executor::task]
async fn run_rtc_and_persistence_task(
    spi: PioSpiDs1302<
        'static,
        embassy_rp::peripherals::PIN_12,
        PIO0,
        0,
    >,
) -> ! {
    let publisher = SETTINGS_PUBLISHER.publisher().unwrap();
    let settings_receiver = SETTINGS_COMMAND.receiver();
    let time_pub = RTC_PUBLISHER.publisher().unwrap();
    let cmd_receiver = RTC_COMMAND.receiver();
    let mut rtc = Ds1302::new(spi);
    info!("enable RTC write");
    // Read time
    rtc.enable_write().await;
    let s = rtc.read_seconds().await;
    rtc.write_seconds(s).await;
    
    let turns_h = rtc.read_ram(0).await as u16;
    let mut turns_per_day = (turns_h << 8) + rtc.read_ram(1).await as u16;
    if turns_per_day == 0 {turns_per_day = 600 };
    let x = turns_per_day - (turns_per_day/100) * 100;
    if x != 0 && x != 50 {turns_per_day -= x;}

    let mut m = rtc.read_ram(2).await;
    if m > 2{m = 2};
    let mode = TurningMode::from_u8(m);
    let mut settings = Settings {
        turns_per_day,
        mode,
        reserved: 0,
    };
    loop {
        Timer::after_secs(1).await;
        let h = rtc.read_hour().await;
        let m = rtc.read_minutes().await;
        let s = rtc.read_seconds().await;
        info!("{:02}:{:02}:{:02}",h,m,s);
        let mut time = WwTime::new(h,m,s);
        if let Ok(cmd) = cmd_receiver.try_receive() {
            match cmd {
                RtcCommand::HourUp => {
                    time.hour += 1;
                    if time.hour > 23 {
                        time.hour = 0;
                    }
                    rtc.write_hour(time.hour).await;
                }
                RtcCommand::HourDown => {
                    if time.hour == 0 {
                        time.hour = 23;
                    } else {
                        time.hour -= 1;
                    };
                    rtc.write_hour(time.hour).await;
                }
                RtcCommand::MinuteUp => {
                    time.minute += 1;
                    if time.minute > 59 {
                        time.minute = 0;
                    }
                    rtc.write_minutes(time.minute).await;
                }
                RtcCommand::MinuteDown => {
                    if time.minute == 0 {
                        time.minute = 59;
                    } else {
                        time.minute -= 1;
                    }
                    rtc.write_minutes(time.minute).await;
                }
                RtcCommand::SetNewTime(h,m) => {
                    time.hour = h;
                    time.minute = m;
                    rtc.write_hour(time.hour).await;
                    rtc.write_minutes(time.minute).await;
            
                }
            }
        } 
        time_pub.publish(time.clone()).await;
        if let Ok(cmd) = settings_receiver.try_receive(){
            match cmd {
                FlashCmd::SetSettings(s) => {
                    info!(
                        "SetSettings {} ({})",
                        s.turns_per_day,
                        match s.mode {
                            TurningMode::CW => "cw",
                            TurningMode::CCW => "ccw",
                            TurningMode::Alternate => "alternating",
                        }
                    );
                    settings = s;
                    let turn_h = ((settings.turns_per_day >> 8) & 0xFF) as u8;
                    let turn_l = (settings.turns_per_day & 0xFF) as u8;
                    rtc.write_ram(0, turn_h).await;
                    rtc.write_ram(1, turn_l).await;
                    rtc.write_ram(2, settings.mode as u8).await;
                 } // Set new settings in flash
                FlashCmd::SendSettings => { // Nothing right now.
                } // Read setting from flash and publish
            }
            info!(
                "Publish settings {} ({})",
                settings.turns_per_day,
                match settings.mode {
                    TurningMode::CW => "cw",
                    TurningMode::CCW => "ccw",
                    TurningMode::Alternate => "alternating",
                }
            );
            publisher.publish(settings.clone()).await;
        }
    }
}

#[embassy_executor::task]
async fn run_motor_control(
    mut led: Output<'static, PIN_16>,
    in1: Output<'static, PIN_6>,
    in2: Output<'static, PIN_7>,
    in3: Output<'static, PIN_8>,
    in4: Output<'static, PIN_9>,
) -> ! {
    let mut sub_settings = SETTINGS_PUBLISHER.subscriber().unwrap();
    let settings = sub_settings.next_message_pure().await;
    let cmd_receiver = MOTOR_CMD_CHANNEL.receiver();
    let mut sub_time = RTC_PUBLISHER.subscriber().unwrap();
    let time = sub_time.next_message_pure().await;
    let mut turning_state = TurnState::new(
        ULN2003::new(in1, in2, in3, in4, Some(Delay {})),
        RState::Stopped,
        settings,
        time,
    );
    turning_state.report_turns();
    info!("Motor task got settings. Start loop");
    loop {
        if turning_state.tick_turn() {
            Timer::after_millis(3).await;
        } else {
            Timer::after_millis(20).await;
        }

        if let Some(w_message) = sub_settings.try_next_message() {
            match w_message {
                embassy_sync::pubsub::WaitResult::Lagged(_) => {}
                embassy_sync::pubsub::WaitResult::Message(s) => {
                    turning_state.new_settings(s);
                    info!(
                        "Motor got new settings {} revs/day {:?}",
                        s.turns_per_day,
                        match s.mode {
                            TurningMode::CW => "cw",
                            TurningMode::CCW => "ccw",
                            TurningMode::Alternate => "alternating",
                        }
                    );
                }
            }
        }
        if let Some(t) = sub_time.try_next_message_pure() {
            turning_state.time(t);
        }
        if let Ok(cmd) = cmd_receiver.try_receive() {
            match cmd {
                MotorCommand::Start => {
                    turning_state.start();
                    led.set_high();
                }
                MotorCommand::Stop => {
                    turning_state.stop();
                    led.set_low();
                }
                MotorCommand::Step(dir) => {
                    turning_state.step(dir);
                }
                MotorCommand::Park => {
                    turning_state.park();
                }
            }
        }
    }
}

// Key handling
#[embassy_executor::task]
async fn run_key_task(
    up: Input<'static, PIN_18>,
    down: Input<'static, PIN_19>,
    enter: Input<'static, PIN_20>,
    back: Input<'static, PIN_21>,
) -> ! {
    let mut key_pressed: Option<Key> = None;
    loop {
        match &key_pressed {
            Some(key) => {
                if up.get_level() == Level::High
                    && down.get_level() == Level::High
                    && enter.get_level() == Level::High
                    && back.get_level() == Level::High
                {
                    let k = key.clone();
                    key_pressed = None;
                    KEY_CHANNEL.send(k).await;
                }
            }
            None => {
                if up.get_level() == Level::Low {
                    key_pressed = Some(Key::Up);
                } else if down.get_level() == Level::Low {
                    key_pressed = Some(Key::Down);
                } else if enter.get_level() == Level::Low {
                    key_pressed = Some(Key::Enter);
                } else if back.get_level() == Level::Low {
                    key_pressed = Some(Key::Back);
                }
            }
        }
        Timer::after_millis(50).await; // this should eliminate switch bounce
    }
}

static MENU: [&str; 4] = [
    "    Turns/day",
    "  Turning mode",
    "    Set Time",
    " Adjust parking",
];

static ADJUST: [&str; 2] = ["    Large step", "     Small step"];

#[embassy_executor::task]
async fn run_show_menu_task(
    mut display: Ssd1306<
        I2CInterface<i2c::I2c<'static, I2C1, embassy_rp::i2c::Async>>,
        DisplaySize128x64,
        BufferedGraphicsMode<DisplaySize128x64>,
    >,
) {
    // Display a splash screen
    let splash = include_bytes!("../splash1.bmp");
    let img = Bmp::from_slice(splash).unwrap();
    display.init().unwrap();
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_7X13)
        .text_color(BinaryColor::On)
        .build();
    let _ = display.clear(BinaryColor::Off);
    Image::new(&img, Point::zero()).draw(&mut display).unwrap();
    display.flush().unwrap();

    let mut time_sub = RTC_PUBLISHER.subscriber().unwrap();

    let mut lines = [FmtBuf::new(); 4];

    let mut time_h = 0;
    let mut time_m = 0;
    let mut time_s = 0;
    let mut turns: i32 = 0;
    let mut total: u16 = 0;
    let mut show_action: Option<Action> = None;
    loop {
        match select::select(ACTION.receive(), time_sub.next_message_pure()).await {
            Either::First(action) => {
                show_action = Some(action);
            }
            Either::Second(t) => {
                time_h = t.hour;
                time_m = t.minute;
                time_s = t.second;
            }
        }
        if let Some(action) = &show_action {
            match action {
                Action::DoNothing => {}
                Action::SetTime { hour, min } => {
                    time_h = *hour;
                    time_m = *min;
                }
                Action::StopTurning => {}
                Action::StartTurning => {}
                Action::StepMotor(_) => {}
                Action::HourUp | Action::HourDown | Action::MinuteUp | Action::MinuteDown => {}
                Action::UpdateSettings(_settings) => {}
                Action::ShowMenu(m) => {
                    lines[0].reset();
                    lines[1].reset();
                    lines[2].reset();
                    lines[3].reset();
                    match m {
                        MenuState::StoppedShowTimeAndTurns => {
                            if TURN_COUNT.signaled() {
                                (turns, total) = TURN_COUNT.wait().await;
                            }
                            core::write!(lines[0], "     Stopped").unwrap();
                            core::write!(lines[1], "").unwrap();
                            core::write!(
                                lines[2],
                                "     {:02}:{:02}:{:02}",
                                time_h,
                                time_m,
                                time_s
                            )
                            .unwrap();
                            core::write!(lines[3], "   ({:>3} of {:>3})", turns, total).unwrap();
                        }
                        MenuState::RunningShowTimeAndTurns => {
                            if TURN_COUNT.signaled() {
                                (turns, total) = TURN_COUNT.wait().await;
                            }
                            core::write!(lines[0], "     Running").unwrap();
                            core::write!(lines[1], "").unwrap();
                            core::write!(
                                lines[2],
                                "     {:02}:{:02}:{:02}",
                                time_h,
                                time_m,
                                time_s
                            )
                            .unwrap();
                            core::write!(lines[3], "   ({:>3} of {:>3})", turns, total).unwrap();
                        }
                        MenuState::Menu(item) => {
                            core::write!(lines[0], "     Settings").unwrap();
                            core::write!(lines[1], "").unwrap();
                            core::write!(lines[2], "{}", MENU[*item as usize]).unwrap();
                            core::write!(lines[3], "").unwrap();
                        }
                        MenuState::SettingTurnsPerDay(turns_p_day) => {
                            total = *turns_p_day;
                            core::write!(lines[0], "     Settings").unwrap();
                            core::write!(lines[1], "").unwrap();
                            core::write!(lines[2], "{}", MENU[0]).unwrap();
                            core::write!(lines[3], "     {}", turns_p_day).unwrap();
                        }
                        MenuState::SettingMode(t) => {
                            core::write!(lines[0], "     Settings").unwrap();
                            core::write!(lines[1], "").unwrap();
                            core::write!(lines[2], "{}", MENU[1]).unwrap();
                            core::write!(lines[3], " {:?}", t).unwrap();
                        }
                        MenuState::SettingTimeHour(h,m) => {
                            core::write!(lines[0], "     Settings").unwrap();
                            core::write!(lines[1], "{}", MENU[2]).unwrap();
                            core::write!(lines[2], "   {:02}:{:02}", h, m).unwrap();
                            core::write!(lines[3], "   --",).unwrap();
                        }
                        MenuState::SetTimeMinute(h,m) => {
                            core::write!(lines[0], "     Settings").unwrap();
                            core::write!(lines[1], "{}", MENU[2]).unwrap();
                            core::write!(lines[2], "   {:02}:{:02}", h, m).unwrap();
                            core::write!(lines[3], "      --",).unwrap();
                        }
                        MenuState::AdjustParking(pos) => {
                            core::write!(lines[0], "     Settings").unwrap();
                            core::write!(lines[1], "").unwrap();
                            core::write!(lines[2], "{}", MENU[3]).unwrap();
                            core::write!(lines[3], "{}", ADJUST[*pos as usize]).unwrap();
                        }
                        MenuState::AdjustParkingLarge => {
                            core::write!(lines[0], "     Settings").unwrap();
                            core::write!(lines[1], "{}", MENU[3]).unwrap();
                            core::write!(lines[2], "    large step",).unwrap();
                            core::write!(lines[3], " (use up / down)",).unwrap();
                        }
                        MenuState::AdjustParkingSmall => {
                            core::write!(lines[0], "     Settings").unwrap();
                            core::write!(lines[1], "{}", MENU[3]).unwrap();
                            core::write!(lines[2], "    small step",).unwrap();
                            core::write!(lines[3], " (use up / down)",).unwrap();
                        }
                    }
                    let _ = display.clear(BinaryColor::Off);

                    for i in 0..4 {
                        let point = Point::new(0, 15 * i);
                        Text::with_baseline(
                            lines[i as usize].as_str(),
                            point,
                            text_style,
                            Baseline::Top,
                        )
                        .draw(&mut display)
                        .unwrap();
                    }
                    display.flush().unwrap();
                }
            }
        }
    }
}

// Enums and structs

// Settings memory
#[derive(Copy, Clone)]
pub enum FlashCmd {
    SetSettings(Settings),
    SendSettings,
}

#[derive(Copy, Clone)]
pub struct Settings {
    pub turns_per_day: u16,
    pub mode: TurningMode,
    pub reserved: u8,
}

// Motor
#[derive(Copy, Clone)]
pub enum MotorCommand {
    Start,
    Stop,
    Step(Direction),
    Park,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum TurningMode {
    CW = 0,
    CCW = 1,
    Alternate = 2,
}

impl TurningMode{
    pub fn from_u8(val: u8) -> Self{
        match val {
            0 => TurningMode::CW,
            1 => TurningMode::CCW,
            2 => TurningMode::Alternate,
            _ => panic!("Illegal value for TurningMode"),
        }
    }
}

#[derive(Clone, Copy)]
pub enum Direction {
    CCW(i32),
    CW(i32),
}

// running states

pub enum RState {
    Turn { turns: i32 },
    Paus,
    DoneToday,
    Stopped,
}

//pub enum RAction {
//}

pub struct TurnState {
    state: RState,
    turns_left_today: i32,
    settings: Settings,
    current_time: WwTime,
    next_time_to_go: WwTime,
    next_direction: Direction,
    ctrl: MotorControl,
    pos: i32,
}
const TICKS_PER_REV: i32 = 4096;

impl TurnState {
    pub fn new(ctrl: MotorControl, state: RState, set: Settings, now: WwTime) -> Self {
        let next_direction = match set.mode {
            TurningMode::CW => Direction::CW(0),
            TurningMode::CCW | TurningMode::Alternate => Direction::CCW(0),
        };

        TurnState {
            state,
            settings: set,
            turns_left_today: set.turns_per_day as i32,
            current_time: now,
            next_time_to_go: now,
            next_direction,
            ctrl,
            pos: 0,
        }
    }
    pub fn start(&mut self) {
        self.state = RState::Paus;
        info!("State Paus");
        self.check_time_to_go();
    }
    pub fn stop(&mut self) {
        self.state = RState::Stopped;
        info!("State Stopped");
    }
    // poll this from task loop. If true returned the wtach is turning so delay shall be short.
    // When false is returned a linger delay can be used to not use to much CPU.
    pub fn tick_turn(&mut self) -> bool {
        match &mut self.state {
            RState::Turn { turns } => {
                let _ = self.ctrl.step();
                self.pos += 1;
                if self.pos >= TICKS_PER_REV {
                    self.pos = 0;
                    *turns -= 1;
                    self.turns_left_today -= 1;
                    let response = if self.turns_left_today <= 0 {
                        self.state = RState::DoneToday;
                        info!("State DoneToday");
                        false
                    } else {
                        if *turns <= 0 {
                            if self.settings.mode == TurningMode::Alternate {
                                match self.next_direction {
                                    Direction::CCW(_) => self.set_next_dir(Direction::CW(0)),
                                    Direction::CW(_) => self.set_next_dir(Direction::CCW(0)),
                                }
                            }
                            self.next_time_to_go =
                                self.current_time.clone_add_minutes(PAUSE_MINUTES);
                            self.state = RState::Paus;
                            info!("State Paus");
                            false
                        } else {
                            true
                        }
                    };
                    self.report_turns();
                    response
                } else {
                    true
                }
            }
            RState::Paus => self.check_time_to_go(),
            RState::DoneToday => false,
            RState::Stopped => false,
        }
    }
    pub fn new_settings(&mut self, set: Settings) {
        self.settings = set;
    }
    pub fn restart_new_day(&mut self) {
        if let RState::Turn { .. } = self.state {
            self.state = RState::Paus;
            self.park();
        }
        self.turns_left_today = self.settings.turns_per_day as i32;
        self.next_time_to_go.hour = START_HOUR;
        self.next_time_to_go.minute = START_MINUTE;
        self.report_turns();
    }
    pub fn time(&mut self, t: WwTime) {
        let t0 = self.current_time;
        self.current_time = t;
        if t0.hour > t.hour {
            // A new day
            self.restart_new_day();
        }
        self.check_time_to_go();
    }
    pub fn step(&mut self, dir: Direction) {
        match self.state {
            RState::Stopped => match dir {
                Direction::CCW(n) => {
                    info!("Step {} steps ccw", n);
                    if let Direction::CW(_) = self.next_direction {
                        self.set_next_dir(dir);
                    }
                    match if n == 1 {
                        self.ctrl.step()
                    } else {
                        self.ctrl.step_for(n, 5)
                    } {
                        Ok(_) => {
                            self.pos = 0;
                        }
                        Err(_) => {
                            error!("Error stepping");
                        }
                    };
                }
                Direction::CW(n) => {
                    info!("Stepp {} steps cw", n);
                    if let Direction::CCW(_) = self.next_direction {
                        self.set_next_dir(dir);
                    }
                    match if n == 1 {
                        self.ctrl.step()
                    } else {
                        self.ctrl.step_for(n, 5)
                    } {
                        Ok(_) => {
                            self.pos = 0;
                        }
                        Err(_) => {
                            error!("Error stepping");
                        }
                    };
                }
            },
            _ => {}
        }
    }
    pub fn park(&mut self) {
        if self.pos != 0 {
            let n = TICKS_PER_REV - self.pos;
            let _ = self.ctrl.step_for(n, 1);
            self.pos = 0;
            self.turns_left_today -= 1;
            self.report_turns();
        }
    }
    pub fn report_turns(&self) {
        TURN_COUNT.signal((
            self.settings.turns_per_day as i32 - self.turns_left_today,
            self.settings.turns_per_day,
        ));
    }
    fn check_time_to_go(&mut self) -> bool {
        match self.state {
            RState::Turn { .. } => true,
            RState::Paus => {
                if self.turns_left_today <= 0 {
                    self.state = RState::DoneToday;
                    false
                } else if self.current_time.hour >= self.next_time_to_go.hour
                    && self.current_time.minute >= self.next_time_to_go.minute
                {
                    let turns = if self.turns_left_today < 10 {
                        self.turns_left_today
                    } else {
                        10
                    };
                    self.state = RState::Turn { turns };
                    info!("State Turn({})", turns);
                    true
                } else {
                    false
                }
            }
            RState::DoneToday => false,
            RState::Stopped => false, // never time to gon when stopped.
        }
    }
    fn set_next_dir(&mut self, dir: Direction) {
        match dir {
            Direction::CCW(_) => {
                info!("Set dir CCW");
                self.ctrl.set_direction(uln2003::Direction::Normal);
                self.next_direction = Direction::CCW(0);
            }
            Direction::CW(_) => {
                info!("Set dir CW");
                self.ctrl.set_direction(uln2003::Direction::Reverse);
                self.next_direction = Direction::CW(0);
            }
        }
    }
}

// Real Time Clock
#[derive(Copy, Clone)]
pub enum RtcCommand {
    HourUp,
    HourDown,
    MinuteUp,
    MinuteDown,
    SetNewTime(u8,u8),
}

#[derive(Copy, Clone)]
pub struct WwTime {
    hour: u8,
    minute: u8,
    second: u8,
}

impl WwTime {
    pub fn add_second(&mut self) {
        self.second += 1;
        if self.second >= 60 {
            self.second = 0;
            self.minute += 1;
            if self.minute >= 60 {
                self.minute = 0;
                self.hour += 1;
                if self.hour >= 24 {
                    self.hour = 0;
                }
            }
        }
    }

    fn clone_add_minutes(&self, arg: u8) -> WwTime {
        let mut cl = self.clone();
        cl.minute += arg;
        if cl.minute >= 60 {
            cl.minute = cl.minute - 60;
            cl.hour += 1;
            if cl.hour >= 24 {
                cl.hour = 0;
            }
        }
        cl
    }
    
    fn new(hour: u8, minute: u8, second: u8) -> Self {
        Self { hour, minute, second }
    }
}

// Keys
#[derive(Copy, Clone)]
pub enum Key {
    Up,
    Down,
    Enter,
    Back,
}

// Menues
const MENU_MAX: i32 = 3;
#[derive(Copy, Clone)]
pub enum MenuState {
    StoppedShowTimeAndTurns,
    RunningShowTimeAndTurns,
    // pos in menu. 0-Turns/day, 1-Turning mode, 2-Set time, 3-Adjust parking
    Menu(i32),
    SettingTurnsPerDay(u16),
    SettingMode(TurningMode),
    SettingTimeHour(u8,u8),
    SetTimeMinute(u8,u8),
    AdjustParking(i32),
    AdjustParkingLarge,
    AdjustParkingSmall,
}

#[derive(Clone, Copy)]
pub enum Action {
    DoNothing,
    SetTime { hour: u8, min: u8 },
    UpdateSettings(Settings),
    StopTurning,
    StartTurning,
    StepMotor(Direction),
    ShowMenu(MenuState),
    HourUp,
    HourDown,
    MinuteUp,
    MinuteDown,
}

// State machine, reacts to keys and returns actions
pub struct AppStateHandler {
    pub state: MenuState,
    pub settings: Settings,
    pub time: WwTime,
}

impl AppStateHandler {
    pub fn new<>(settings: &Settings,
        time: WwTime ) -> Self {
        AppStateHandler {
            state: MenuState::StoppedShowTimeAndTurns,
            settings: *settings,
            time,
        }
    }

    pub fn get_state(&self) -> MenuState {
        self.state.clone()
    }
    pub fn set_time(&mut self, t:WwTime ){
        self.time = t;
    }
    pub fn key_pressed(&mut self, key: Key) -> Action {
        match self.state {
            MenuState::StoppedShowTimeAndTurns => match key {
                Key::Up => {
                    self.state = MenuState::RunningShowTimeAndTurns;
                    Action::StartTurning
                }
                Key::Down => Action::DoNothing,
                Key::Enter => {
                    self.state = MenuState::Menu(0);
                    Action::ShowMenu(self.state.clone())
                }
                Key::Back => Action::DoNothing,
            },
            MenuState::RunningShowTimeAndTurns => match key {
                Key::Up => {
                    self.state = MenuState::StoppedShowTimeAndTurns;
                    Action::StopTurning
                }
                Key::Down => Action::DoNothing,
                Key::Enter => Action::DoNothing,
                Key::Back => Action::DoNothing,
            },
            MenuState::Menu(m) => match key {
                Key::Up => {
                    let mut next = m - 1;
                    if next < 0 {
                        next = MENU_MAX;
                    }
                    self.state = MenuState::Menu(next);
                    Action::ShowMenu(self.state.clone())
                }
                Key::Down => {
                    let mut next = m + 1;
                    if next > MENU_MAX {
                        next = 0
                    };
                    self.state = MenuState::Menu(next);
                    Action::ShowMenu(self.state.clone())
                }
                Key::Enter => match m {
                    0 => {
                        self.state = MenuState::SettingTurnsPerDay(self.settings.turns_per_day);
                        Action::ShowMenu(self.state.clone())
                    }
                    1 => {
                        self.state = MenuState::SettingMode(self.settings.mode);
                        Action::ShowMenu(self.state.clone())
                    }
                    2 => {                        
                        self.state = MenuState::SettingTimeHour(self.time.hour,self.time.minute);
                        Action::ShowMenu(self.state.clone())
                    }
                    3 => {
                        self.state = MenuState::AdjustParking(0);
                        Action::ShowMenu(self.state.clone())
                    }
                    _ => Action::DoNothing,
                },
                Key::Back => {
                    self.state = MenuState::StoppedShowTimeAndTurns;
                    Action::ShowMenu(self.state.clone())
                }
            },
            MenuState::SettingTurnsPerDay(turns) => {
                match key {
                    Key::Up => {
                        self.state = MenuState::SettingTurnsPerDay(turns + 50);
                    }
                    Key::Down => {
                        let t = if turns - 50 < 50 { 50 } else {turns - 50};
                        self.state = MenuState::SettingTurnsPerDay(t);
                    }
                    Key::Enter => {
                        self.state = MenuState::Menu(0);
                        self.settings.turns_per_day = turns;
                        return Action::UpdateSettings(self.settings.clone());
                    }
                    Key::Back => {
                        self.state = MenuState::Menu(0);
                    }
                };
                Action::ShowMenu(self.state)
            }
            MenuState::SettingMode(mode) => {
                match key {
                    Key::Up => {
                        self.state = MenuState::SettingMode(match mode {
                            TurningMode::CW => TurningMode::CCW,
                            TurningMode::CCW => TurningMode::Alternate,
                            TurningMode::Alternate => TurningMode::CW,
                        });
                    }
                    Key::Down => {
                        self.state = MenuState::SettingMode(match mode {
                            TurningMode::CW => TurningMode::Alternate,
                            TurningMode::CCW => TurningMode::CW,
                            TurningMode::Alternate => TurningMode::CCW,
                        });
                    }
                    Key::Enter => {
                        self.state = MenuState::Menu(1);
                        self.settings.mode = mode;
                        return Action::UpdateSettings(self.settings.clone());
                    }
                    Key::Back => {
                        self.state = MenuState::Menu(1);
                    }
                };
                Action::ShowMenu(self.state)
            }
            MenuState::SettingTimeHour(h,m) => match key {
                Key::Up => {
                    let hn= if h == 23 {0}else{h+1};
                    self.state = MenuState::SettingTimeHour( hn,m);
                    Action::ShowMenu(self.state)
                }
                Key::Down => {
                    let hn = if h == 0 {23}else{h-1};
                    self.state = MenuState::SettingTimeHour( hn,m);
                    Action::ShowMenu(self.state)
                }
                Key::Enter => {
                    self.state = MenuState::SetTimeMinute(h,m);
                    Action::ShowMenu(self.state)
                }
                Key::Back => {
                    self.state = MenuState::Menu(2);
                    Action::ShowMenu(self.state)
                }
            }
            MenuState::SetTimeMinute(h,m) => match key {
                Key::Up => {
                    let mn = if m == 59 {0}else{m+1};
                    self.state = MenuState::SetTimeMinute(h, mn);
                    Action::ShowMenu(self.state)
                }
                Key::Down => {
                    let mn = if m == 0 {59}else{m-1};
                    self.state = MenuState::SetTimeMinute(h, mn);
                    Action::ShowMenu(self.state)
                }
                Key::Enter => {
                    self.state = MenuState::Menu(2);
                    Action::SetTime { hour: h, min: m }
                }
                Key::Back => {
                    self.state = MenuState::Menu(2);
                    Action::ShowMenu(self.state)
                }
            },
            MenuState::AdjustParking(pos) => {
                match key {
                    Key::Up => self.state = MenuState::AdjustParking(if pos == 0 { 1 } else { 0 }),
                    Key::Down => {
                        self.state = MenuState::AdjustParking(if pos == 0 { 1 } else { 0 })
                    }
                    Key::Enter => match pos {
                        0 => {
                            self.state = MenuState::AdjustParkingLarge;
                        }
                        1 => {
                            self.state = MenuState::AdjustParkingSmall;
                        }
                        _ => {}
                    },
                    Key::Back => {
                        self.state = MenuState::Menu(3);
                    }
                };
                Action::ShowMenu(self.state)
            }
            MenuState::AdjustParkingLarge => match key {
                Key::Up => Action::StepMotor(Direction::CCW(100)),
                Key::Down => Action::StepMotor(Direction::CW(100)),
                Key::Enter => Action::DoNothing,
                Key::Back => {
                    self.state = MenuState::AdjustParking(0);
                    Action::ShowMenu(self.state.clone())
                }
            },
            MenuState::AdjustParkingSmall => match key {
                Key::Up => Action::StepMotor(Direction::CCW(1)),
                Key::Down => Action::StepMotor(Direction::CW(1)),
                Key::Enter => Action::DoNothing,
                Key::Back => {
                    self.state = MenuState::AdjustParking(1);
                    Action::ShowMenu(self.state.clone())
                }
            },
        }
    }
}
