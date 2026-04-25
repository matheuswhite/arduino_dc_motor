use ufmt::{uwrite, uwriteln, uWrite};

pub fn write_fixed_f32<W>(writer: &mut W, value: f32, decimals: u32) -> Result<(), W::Error>
where
    W: uWrite + ?Sized,
{
    write_fixed_u32(writer, value < 0.0, value.abs(), decimals)
}

fn write_fixed_u32<W>(writer: &mut W, is_negative: bool, magnitude: f32, decimals: u32) -> Result<(), W::Error>
where
    W: uWrite + ?Sized,
{
    let scale = decimal_scale_u32(decimals);
    let scaled = (magnitude * scale as f32) as u32;

    write_fixed_parts(writer, is_negative, (scaled / scale) as u64, (scaled % scale) as u64, decimals)
}

fn write_fixed_parts<W>(writer: &mut W, is_negative: bool, integer: u64, fraction: u64, decimals: u32) -> Result<(), W::Error>
where
    W: uWrite + ?Sized,
{
    if is_negative {
        uwrite!(writer, "-")?;
    }

    uwrite!(writer, "{}", integer)?;

    if decimals == 0 {
        return Ok(());
    }

    uwrite!(writer, ".")?;

    let mut divisor = decimal_divisor(decimals);
    while divisor > 0 {
        if fraction < divisor {
            uwrite!(writer, "0")?;
        } else {
            break;
        }

        divisor /= 10;
    }

    uwrite!(writer, "{}", fraction)
}

fn decimal_scale_u32(decimals: u32) -> u32 {
    let mut scale = 1;
    let mut digits = 0;

    while digits < decimals {
        scale *= 10;
        digits += 1;
    }

    scale
}

fn decimal_divisor(decimals: u32) -> u64 {
    let mut divisor = 1;
    let mut digits = 1;

    while digits < decimals {
        divisor *= 10;
        digits += 1;
    }

    divisor
}