#[cfg(feature = "nalgebra")]
use nalgebra::{Vector2, Vector3, Vector4};
#[cfg(feature = "vqm")]
use vqm::{Vector2, Vector3, Vector4};

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
macro_rules! nalgebra_filter_mul_add {
    ($vector:ty, $scalar:ty) => {
        impl FilterMulAdd<$scalar> for $vector {
            fn filter_mul_add(self, a: $scalar, b: Self) -> Self {
                self * a + b
            }
        }
        impl FilterMulAdd<$vector> for $vector {
            fn filter_mul_add(self, a: $vector, b: Self) -> Self {
                self.component_mul(&a) + b
            }
        }
    };
}

#[cfg(feature = "vqm")]
macro_rules! vqm_filter_mul_add {
    ($vector:ty, $scalar:ty) => {
        impl FilterMulAdd<$scalar> for $vector {
            fn filter_mul_add(self, a: $scalar, b: Self) -> Self {
                num_traits::MulAdd::mul_add(self, a, b)
            }
        }
        impl FilterMulAdd<$vector> for $vector {
            fn filter_mul_add(self, a: $vector, b: Self) -> Self {
                num_traits::MulAdd::mul_add(self, a, b)
            }
        }
    };
}

#[cfg(feature = "nalgebra")]
nalgebra_filter_mul_add!(Vector2<f32>, f32);
#[cfg(feature = "nalgebra")]
nalgebra_filter_mul_add!(Vector3<f32>, f32);
#[cfg(feature = "nalgebra")]
nalgebra_filter_mul_add!(Vector4<f32>, f32);

#[cfg(feature = "nalgebra")]
nalgebra_filter_mul_add!(Vector2<f64>, f64);
#[cfg(feature = "nalgebra")]
nalgebra_filter_mul_add!(Vector3<f64>, f64);
#[cfg(feature = "nalgebra")]
nalgebra_filter_mul_add!(Vector4<f64>, f64);

#[cfg(feature = "vqm")]
vqm_filter_mul_add!(Vector2<f32>, f32);
#[cfg(feature = "vqm")]
vqm_filter_mul_add!(Vector3<f32>, f32);
#[cfg(feature = "vqm")]
vqm_filter_mul_add!(Vector4<f32>, f32);

#[cfg(feature = "vqm")]
vqm_filter_mul_add!(Vector2<f64>, f64);
#[cfg(feature = "vqm")]
vqm_filter_mul_add!(Vector3<f64>, f64);
#[cfg(feature = "vqm")]
vqm_filter_mul_add!(Vector4<f64>, f64);
