#[cfg(feature = "nalgebra")]
use nalgebra::Vector3;
#[cfg(feature = "vqm")]
use vqm::Vector3;

pub trait FilterMulAdd<R, A = Self> {
    #[must_use]
    fn filter_mul_add(self, a: R, b: A) -> Self;
}

impl FilterMulAdd<f32> for f32 {
    fn filter_mul_add(self, a: f32, b: Self) -> Self {
        num_traits::MulAdd::mul_add(self, a, b)
    }
}

impl FilterMulAdd<f64> for f64 {
    fn filter_mul_add(self, a: f64, b: Self) -> Self {
        num_traits::MulAdd::mul_add(self, a, b)
    }
}

#[cfg(feature = "nalgebra")]
impl FilterMulAdd<f32> for Vector3<f32> {
    fn filter_mul_add(self, a: f32, b: Self) -> Self {
        self * a + b
    }
}

#[cfg(feature = "nalgebra")]
impl FilterMulAdd<Vector3<f32>> for Vector3<f32> {
    fn filter_mul_add(self, a: Vector3<f32>, b: Self) -> Self {
        self.component_mul(&a) + b
    }
}

#[cfg(feature = "vqm")]
impl FilterMulAdd<f32> for Vector3<f32> {
    fn filter_mul_add(self, a: f32, b: Self) -> Self {
        num_traits::MulAdd::mul_add(self, a, b)
    }
}

#[cfg(feature = "vqm")]
impl FilterMulAdd<Vector3<f32>> for Vector3<f32> {
    fn filter_mul_add(self, a: Vector3<f32>, b: Self) -> Self {
        num_traits::MulAdd::mul_add(self, a, b)
    }
}
