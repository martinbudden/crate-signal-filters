#![allow(unused)]
use num_traits::Zero;
/*
#[repr(C, align(16))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vector3df32 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}


#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pt1FilterSimd {
    state: Vector3df32,
    k: f32,
}

/// Default is k = 1.0, which is passthrough
impl Default for Pt1FilterSimd
{
    fn default() -> Self {
        Self::new(1.0)
    }
}

impl Pt1FilterSimd
{
    pub fn new(k: f32) -> Self {
        Self { state: Vector3df32::default(), k }
    }

    fn reset(&mut self) {
        self.state = Vector3df32::default();
    }

    fn update(&mut self, input: Vector3df32) -> Vector3df32 {
        self.state  += (input - self.state) * self.k;
        self.state
    }
}
*/
