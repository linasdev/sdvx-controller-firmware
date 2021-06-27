pub struct SdvxStatus {
    pub start_pressed: bool,
    pub button1_pressed: bool,
    pub button2_pressed: bool,
    pub button3_pressed: bool,
    pub button4_pressed: bool,
    pub fx_l_pressed: bool,
    pub fx_r_pressed: bool,
    pub rotary1_rotated_ccw: bool,
    pub rotary1_rotated_cw: bool,
    pub rotary2_rotated_ccw: bool,
    pub rotary2_rotated_cw: bool,
}

impl SdvxStatus {
    pub fn new() -> Self {
        SdvxStatus {
            start_pressed: false,
            button1_pressed: false,
            button2_pressed: false,
            button3_pressed: false,
            button4_pressed: false,
            fx_l_pressed: false,
            fx_r_pressed: false,
            rotary1_rotated_ccw: false,
            rotary1_rotated_cw: false,
            rotary2_rotated_ccw: false,
            rotary2_rotated_cw: false,
        }
    }
}
