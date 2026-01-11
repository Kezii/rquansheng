pub enum FrequencyBand {
    BandNone = -1,
    Band1_50MHz = 0,
    Band2_108MHz,
    Band3_137MHz,
    Band4_174MHz,
    Band5_350MHz,
    Band6_400MHz,
    Band7_470MHz,
}

impl FrequencyBand {
    pub fn from_frequency(frequency: u32) -> Option<Self> {
        match frequency {
            1800000..=7600000 => Some(Self::Band1_50MHz),
            10800000..=13700000 => Some(Self::Band2_108MHz),
            13700000..=17400000 => Some(Self::Band3_137MHz),
            17400000..=35000000 => Some(Self::Band4_174MHz),
            35000000..=40000000 => Some(Self::Band5_350MHz),
            40000000..=47000000 => Some(Self::Band6_400MHz),
            47000000..=130000000 => Some(Self::Band7_470MHz),
            _ => None,
        }
    }
}
