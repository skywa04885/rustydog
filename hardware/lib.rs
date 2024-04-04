pub enum Servo {
    Joint0,
    Joint1,
    Joint2,
    Joint3,
    Joint4
}

impl Servo {
    /// Get the address of the holding register belonging to the current servo.
    fn target_angle_holding_reg_addr(&self) -> u16 {
        match self {
            &Self::Joint0 => 0x0001,
            &Self::Joint1 => 0x0003,
            &Self::Joint2 => 0x0005,
            &Self::Joint3 => 0x0007,
            &Self::Joint4 => 0x0009,
        }
    }


}

pub struct Hardware {

}

impl Hardware {

}