use crate::PixelBitDepth;

pub fn calc_bcm_value(value: f64) -> PixelBitDepth {
    return (value * PixelBitDepth::MAX as f64) as PixelBitDepth;
}
