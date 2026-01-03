use heapless::String;

use crate::keyboard::{Key, KeyEvent};

#[derive(Clone)]
pub struct Dialer {
    number: String<6>,
}

impl Dialer {
    pub fn new() -> Self {
        Self {
            number: String::<6>::new(),
        }
    }

    pub fn eat_keyboard_event(&mut self, event: KeyEvent) {
        if let KeyEvent::KeyPressed(key) = event {
            match key {
                Key::Num0 => {
                    self.number.push('0');
                }
                Key::Num1 => {
                    self.number.push('1');
                }
                Key::Num2 => {
                    self.number.push('2');
                }
                Key::Num3 => {
                    self.number.push('3');
                }
                Key::Num4 => {
                    self.number.push('4');
                }
                Key::Num5 => {
                    self.number.push('5');
                }
                Key::Num6 => {
                    self.number.push('6');
                }
                Key::Num7 => {
                    self.number.push('7');
                }
                Key::Num8 => {
                    self.number.push('8');
                }
                Key::Num9 => {
                    self.number.push('9');
                }
                Key::Exit => {
                    self.number.clear();
                }
                _ => {}
            }
        }
    }

    pub fn get_as_string(&self) -> String<6> {
        self.number.clone()
    }

    pub fn clear(&mut self) {
        self.number.clear();
    }

    pub fn get_frequency(&mut self) -> Option<u32> {
        if self.number.len() != 6 {
            return None;
        }
        let mut frequency = 0;
        for (i, ch) in self.number.as_str().chars().enumerate() {
            frequency = frequency + (ch as u32 - '0' as u32) * 10_u32.pow(5 - i as u32);
        }

        self.clear();
        Some(frequency)
    }
}
