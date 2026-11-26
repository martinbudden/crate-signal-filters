#[cfg(feature = "nalgebra")]
use nalgebra::Vector3;
#[cfg(feature = "vqm")]
use vqm::Vector3;

pub trait FilterOne {
    fn filter_one() -> Self;
}

impl FilterOne for f32 {
    fn filter_one() -> Self {
        1.0
    }
}

impl FilterOne for f64 {
    fn filter_one() -> Self {
        1.0
    }
}

#[cfg(feature = "nalgebra")]
impl FilterOne for Vector3<f32> {
    fn filter_one() -> Self {
        Self::repeat(1.0)
    }
}

#[cfg(feature = "vqm")]
impl FilterOne for Vector3<f32> {
    fn filter_one() -> Self {
        num_traits::One::one()
    }
}
