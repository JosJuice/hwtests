#include <algorithm>
#include <array>
#include <bit>
#include <gctypes.h>
#include <wiiuse/wpad.h>

#include "Common/CommonTypes.h"
#include "Common/FloatUtils.h"
#include "Common/hwtests.h"

constexpr u64 QNAN = DOUBLE_EXP | DOUBLE_QBIT;

static bool s_debug_prints = false;

struct U128
{
  constexpr U128() = default;
  constexpr U128(u64 value) : upper(0), lower(value) {}

  u64 upper;
  u64 lower;
};

static constexpr bool operator==(const U128& lhs, const U128& rhs)
{
  return lhs.upper == rhs.upper && lhs.lower == rhs.lower;
}

static constexpr bool operator==(const U128& lhs, u64 rhs)
{
  return lhs.upper == 0 && lhs.lower == rhs;
}

static constexpr bool operator==(u64 lhs, const U128& rhs)
{
  return operator==(rhs, lhs);
}

static constexpr U128& operator+=(U128& lhs, const U128& rhs)
{
  const bool carry = __builtin_add_overflow(lhs.lower, rhs.lower, &lhs.lower);
  lhs.upper = lhs.upper + rhs.upper + carry;
  return lhs;
}

static constexpr U128 operator+(U128 lhs, const U128& rhs)
{
  lhs += rhs;
  return lhs;
}

static constexpr U128& operator-=(U128& lhs, const U128& rhs)
{
  const bool overflow = __builtin_sub_overflow(lhs.lower, rhs.lower, &lhs.lower);
  lhs.upper = lhs.upper - rhs.upper - overflow;
  return lhs;
}

static constexpr U128 operator-(U128 lhs, const U128& rhs)
{
  lhs -= rhs;
  return lhs;
}

static constexpr U128 operator-(const U128& value)
{
  return U128(0) - value;
}

static constexpr U128& operator&=(U128& lhs, const U128& rhs)
{
  lhs.upper &= rhs.upper;
  lhs.lower &= rhs.lower;
  return lhs;
}

static constexpr U128 operator&(U128 lhs, const U128& rhs)
{
  lhs &= rhs;
  return lhs;
}

static constexpr U128& operator|=(U128& lhs, const U128& rhs)
{
  lhs.upper |= rhs.upper;
  lhs.lower |= rhs.lower;
  return lhs;
}

static constexpr U128 operator|(U128 lhs, const U128& rhs)
{
  lhs |= rhs;
  return lhs;
}

static constexpr U128 operator~(U128 value)
{
  value.upper = ~value.upper;
  value.lower = ~value.lower;
  return value;
}

static constexpr int countl_zero(const U128& value)
{
  if (value.upper == 0)
    return std::countl_zero(value.lower) + 64;
  else
    return std::countl_zero(value.upper);
}

static U128 Shift128RightWithStickyBit(U128 value, int shift, bool* sticky_bit)
{
  if (shift == 0)
  {
  }
  else if (shift < 64)
  {
    if (((~u64(0) >> (64 - shift)) & value.lower) != 0)
      *sticky_bit = true;

    value.lower = (value.lower >> shift) | (value.upper << (64 - shift));
    value.upper = value.upper >> shift;
  }
  else if (shift < 128)
  {
    if (value.lower != 0 || ((~u64(0) >> (128 - shift)) & value.upper) != 0)
      *sticky_bit = true;

    value.lower = value.upper >> (shift - 64);
    value.upper = 0;
  }
  else
  {
    if (value.lower != 0 || value.upper != 0)
      *sticky_bit = true;

    value.lower = 0;
    value.upper = 0;
  }
  return value;
}

static constexpr U128& operator>>=(U128& value, int shift)
{
  if (shift == 0)
  {
  }
  else if (shift < 64)
  {
    value.lower = (value.lower >> shift) | (value.upper << (64 - shift));
    value.upper = value.upper >> shift;
  }
  else if (shift < 128)
  {
    value.lower = value.upper >> (shift - 64);
    value.upper = 0;
  }
  else
  {
    value.lower = 0;
    value.upper = 0;
  }
  return value;
}

static constexpr U128 operator>>(U128 value, int shift)
{
  value >>= shift;
  return value;
}

static constexpr U128& operator<<=(U128& value, int shift)
{
  if (shift == 0)
  {
  }
  else if (shift < 64)
  {
    value.upper = (value.upper << shift) | (value.lower >> (64 - shift));
    value.lower = value.lower << shift;
  }
  else if (shift < 128)
  {
    value.upper = value.lower << (shift - 64);
    value.lower = 0;
  }
  else
  {
    value.upper = 0;
    value.lower = 0;
  }
  return value;
}

static constexpr U128 operator<<(U128 value, int shift)
{
  value <<= shift;
  return value;
}

static constexpr U128 ShiftLeftTo128(u64 value, int shift)
{
  U128 result;
  if (shift == 0)
  {
    result.upper = 0;
    result.lower = value;
  }
  else if (shift < 64)
  {
    result.upper = value >> (64 - shift);
    result.lower = value << shift;
  }
  else if (shift < 128)
  {
    result.upper = value << (shift - 64);
    result.lower = 0;
  }
  else
  {
    result.upper = 0;
    result.lower = 0;
  }
  return result;
}

static constexpr U128 MultiplyTo128(u64 lhs, u64 rhs)
{
  U128 result = 0;
  while (lhs != 0)
  {
    const int shift = 63 - std::countl_zero(lhs);
    result += ShiftLeftTo128(rhs, shift);
    lhs &= ~(u64(1) << shift);
  }
  return result;
}

// If the most significant 1 isn't at 1 << 105, increment/decrement the exponent
// and shift the mantissa to make it that way.
static void AdjustExponent(int* exponent, U128* mantissa, bool* sticky_bit)
{
  const int leading_zeroes = countl_zero(*mantissa);
  if (leading_zeroes == 128)
  {
    *exponent = 0;
    return;
  }

  const int shift = (128 - 105) - leading_zeroes;
  if (shift > 0)
    *mantissa = Shift128RightWithStickyBit(*mantissa, shift, sticky_bit);
  else if (shift < 0)
    *mantissa <<= -shift;

  *exponent += shift;
}

struct UnpackedFloat
{
  u64 mantissa;
  int exponent;
  bool sign;
};

static UnpackedFloat UnpackFloat(u64 value)
{
  return UnpackedFloat{value & DOUBLE_FRAC,
                       static_cast<int>((value & DOUBLE_EXP) >> DOUBLE_FRAC_WIDTH),
                       static_cast<bool>(value >> DOUBLE_SIGN_SHIFT)};
}

static u64 PackFloat(UnpackedFloat value)
{
  return u64(value.sign ? 1 : 0) << DOUBLE_SIGN_SHIFT |
         u64(value.exponent & 0x7ff) << DOUBLE_FRAC_WIDTH |
         (value.mantissa & DOUBLE_FRAC);
}

static UnpackedFloat Normalize(UnpackedFloat value)
{
  if (value.exponent == 0)
  {
    if (value.mantissa != 0)
    {
      const int shift = std::countl_zero(value.mantissa) - (64 - 53);
      if (s_debug_prints)
        network_printf(fmt::format("Normalizing with shift {}\n", shift).c_str());
      value.mantissa <<= shift;
      value.exponent -= shift - 1;
    }
  }
  else
  {
    value.mantissa |= u64(1) << DOUBLE_FRAC_WIDTH;
  }

  return value;
}

static bool IsNaN(const UnpackedFloat& value)
{
  return value.exponent == 0x7ff && value.mantissa != 0;
}

static bool IsInfinity(const UnpackedFloat& value)
{
  return value.exponent == 0x7ff && value.mantissa == 0;
}

static u64 InfinityWithSign(bool sign)
{
  return DOUBLE_EXP | (u64(sign) << DOUBLE_SIGN_SHIFT);
}

static u64 AddInfinityAndB(u64 infinity, const UnpackedFloat& b_unpacked)
{
  const bool infinity_sign = ((infinity & DOUBLE_SIGN) != 0);
  if (IsInfinity(b_unpacked) && infinity_sign != b_unpacked.sign)
    return QNAN;
  else
    return infinity;
}

static u64 TruncateSingleFraction(u64 value, bool single = true)
{
  if (single)
    value &= 0xffff'ffff'e000'0000;
  return value;
}

static u32 GetRN(u32 fpscr_low_bits)
{
  return fpscr_low_bits & 0x3;
}

static bool GetNI(u32 fpscr_low_bits)
{
  return (fpscr_low_bits & 0x4) != 0;
}

static u64 SoftwareFma(u64 a, u64 b, u64 c, u32 fpscr_low_bits, bool single)
{
  UnpackedFloat a_unpacked = UnpackFloat(a);
  UnpackedFloat b_unpacked = UnpackFloat(b);
  UnpackedFloat c_unpacked = UnpackFloat(c);

  if (s_debug_prints)
  {
    network_printf(fmt::format("a: {:01x} {:03x} {:016x}\n", a_unpacked.sign, a_unpacked.exponent, a_unpacked.mantissa).c_str());
    network_printf(fmt::format("b: {:01x} {:03x} {:016x}\n", b_unpacked.sign, b_unpacked.exponent, b_unpacked.mantissa).c_str());
    network_printf(fmt::format("c: {:01x} {:03x} {:016x}\n", c_unpacked.sign, c_unpacked.exponent, c_unpacked.mantissa).c_str());
  }

  if (IsNaN(a_unpacked))
    return TruncateSingleFraction(a | DOUBLE_QBIT, single);
  else if (IsNaN(b_unpacked))
    return TruncateSingleFraction(b | DOUBLE_QBIT, single);
  else if (IsNaN(c_unpacked))
    return TruncateSingleFraction(c | DOUBLE_QBIT, single);

  const u64 infinity_multiplication_result = InfinityWithSign(a_unpacked.sign ^ c_unpacked.sign);
  if (IsInfinity(a_unpacked) && IsInfinity(c_unpacked))
  {
    return TruncateSingleFraction(AddInfinityAndB(infinity_multiplication_result, b_unpacked), single);
  }
  else if (IsInfinity(a_unpacked))
  {
    if (c_unpacked.exponent == 0 && c_unpacked.mantissa == 0)
      return QNAN;
    else
      return TruncateSingleFraction(AddInfinityAndB(infinity_multiplication_result, b_unpacked), single);
  }
  else if (IsInfinity(c_unpacked))
  {
    if (a_unpacked.exponent == 0 && a_unpacked.mantissa == 0)
      return QNAN;
    else
      return TruncateSingleFraction(AddInfinityAndB(infinity_multiplication_result, b_unpacked), single);
  }
  else if (IsInfinity(b_unpacked))
  {
    return TruncateSingleFraction(InfinityWithSign(b_unpacked.sign), single);
  }

  a_unpacked = Normalize(a_unpacked);
  b_unpacked = Normalize(b_unpacked);
  c_unpacked = Normalize(c_unpacked);

  // Round the C input for single-precision multiplication
  if (single)
  {
    if (c_unpacked.mantissa != 0)
    {
      const u64 round_bit = 0x08000000;
      c_unpacked.mantissa += c_unpacked.mantissa & round_bit;
      c_unpacked.mantissa &= ~(round_bit - 1);

      if ((c_unpacked.mantissa & (u64(1) << (DOUBLE_FRAC_WIDTH + 1))) != 0)
      {
        c_unpacked.mantissa >>= 1;
        c_unpacked.exponent += 1;
      }
    }

    if (s_debug_prints)
      network_printf(fmt::format("{:01x} {:03x} {:016x}\n", c_unpacked.sign, c_unpacked.exponent, c_unpacked.mantissa).c_str());
  }

  // Multiply
  bool sign = a_unpacked.sign ^ c_unpacked.sign;
  int exponent = a_unpacked.exponent + c_unpacked.exponent - 0x3ff;
  U128 mantissa = MultiplyTo128(a_unpacked.mantissa, c_unpacked.mantissa);

  if (s_debug_prints)
    network_printf(fmt::format("{:01x} {:03x} {:016x}{:016x}\n", sign, exponent, mantissa.upper, mantissa.lower).c_str());

  bool sticky_bit = false;
  AdjustExponent(&exponent, &mantissa, &sticky_bit);

  if (s_debug_prints)
    network_printf(fmt::format("{:01x} {:03x} {:016x}{:016x} {:01x}\n", sign, exponent, mantissa.upper, mantissa.lower, sticky_bit).c_str());

  // Add
  if (b_unpacked.mantissa != 0 || mantissa != 0)
  {
    U128 b_shifted_mantissa = ShiftLeftTo128(b_unpacked.mantissa, DOUBLE_FRAC_WIDTH);
    bool b_sticky_bit = false;
    if (b_unpacked.exponent < exponent)
    {
      const int diff = exponent - b_unpacked.exponent;
      b_shifted_mantissa = Shift128RightWithStickyBit(b_shifted_mantissa, diff, &b_sticky_bit);
    }
    else
    {
      const int diff = b_unpacked.exponent - exponent;
      mantissa = Shift128RightWithStickyBit(mantissa, diff, &sticky_bit);
      exponent += diff;
    }

    if (s_debug_prints)
      network_printf(fmt::format("{:01x} {:03x} {:016x}{:016x} {:01x}\n", sign, exponent, mantissa.upper, mantissa.lower, sticky_bit).c_str());

    if (sign == b_unpacked.sign)
    {
      mantissa += b_shifted_mantissa;
      sticky_bit |= b_sticky_bit;
    }
    else
    {
      mantissa -= b_shifted_mantissa;
      if (b_sticky_bit && !sticky_bit)
      {
        mantissa -= 1;
        sticky_bit = true;
      }
    }

    if (s_debug_prints)
      network_printf(fmt::format("{:01x} {:03x} {:016x}{:016x} {:01x}\n", sign, exponent, mantissa.upper, mantissa.lower, sticky_bit).c_str());

    if ((mantissa & (U128(1) << 127)) != 0)
    {
      mantissa = -mantissa;
      sign = !sign;
      if (sticky_bit)
        mantissa -= 1;
    }
  }

  if (s_debug_prints)
    network_printf(fmt::format("{:01x} {:03x} {:016x}{:016x} {:01x}\n", sign, exponent, mantissa.upper, mantissa.lower, sticky_bit).c_str());

  if (mantissa == 0 && sign != b_unpacked.sign)
    sign = GetRN(fpscr_low_bits) == 0x3;

  if (s_debug_prints)
    network_printf(fmt::format("{:01x} {:03x} {:016x}{:016x} {:01x}\n", sign, exponent, mantissa.upper, mantissa.lower, sticky_bit).c_str());

  AdjustExponent(&exponent, &mantissa, &sticky_bit);

  if (s_debug_prints)
    network_printf(fmt::format("{:01x} {:03x} {:016x}{:016x} {:01x}\n", sign, exponent, mantissa.upper, mantissa.lower, sticky_bit).c_str());

  // Denormalize
  const int denormal_threshold = single ? 0x380 : 0;
  if (exponent <= denormal_threshold)
  {
    if (GetNI(fpscr_low_bits))
      return sign ? 0x8000000000000000ULL : 0;

    mantissa = Shift128RightWithStickyBit(mantissa, denormal_threshold + 1 - exponent, &sticky_bit);
    exponent = 0;
  }

  if (s_debug_prints)
    network_printf(fmt::format("{:01x} {:03x} {:016x}{:016x} {:01x}\n", sign, exponent, mantissa.upper, mantissa.lower, sticky_bit).c_str());

  // Round
  const U128 lowest_bit_of_result_mask = U128(1) << ((single ? DOUBLE_FRAC_WIDTH - FLOAT_FRAC_WIDTH : 0) + DOUBLE_FRAC_WIDTH);
  const bool lowest_bit_of_result = (mantissa & lowest_bit_of_result_mask) != 0;
  const U128 guard_bit_mask = lowest_bit_of_result_mask >> 1;
  const bool guard_bit = (mantissa & guard_bit_mask) != 0;
  const U128 additional_sticky_bits_mask = guard_bit_mask - 1;
  const bool additional_sticky_bits = (mantissa & additional_sticky_bits_mask) != 0;
  mantissa &= ~(guard_bit_mask | additional_sticky_bits_mask);
  bool round_up = false;
  bool forbid_infinity = false;
  switch (GetRN(fpscr_low_bits))
  {
  case 0:  // Round to nearest, tie toward even
    round_up = guard_bit && (sticky_bit || additional_sticky_bits || lowest_bit_of_result);
    forbid_infinity = false;
    if (s_debug_prints)
      network_printf(fmt::format("guard {}, additional sticky {}, sticky {}\n", guard_bit, additional_sticky_bits, sticky_bit).c_str());
    break;
  case 1:  // Round toward zero
    round_up = false;
    forbid_infinity = true;
    break;
  case 2:  // Round toward positive infinity
    round_up = !sign && (guard_bit || additional_sticky_bits || sticky_bit);
    forbid_infinity = sign;
    break;
  case 3:  // Round toward negative infinity
    round_up = sign && (guard_bit || additional_sticky_bits || sticky_bit);
    forbid_infinity = !sign;
    break;
  }
  if (round_up)
  {
    mantissa += lowest_bit_of_result_mask;
    if ((mantissa & (U128(1) << 105)) != 0)
    {
      mantissa >>= 1;
      exponent += 1;
    }
    else if (exponent == 0 && (mantissa & (U128(1) << 104)) != 0)
    {
      exponent = 1;
    }
  }

  if (s_debug_prints)
    network_printf(fmt::format("{:01x} {:03x} {:016x}{:016x}\n", sign, exponent, mantissa.upper, mantissa.lower).c_str());

  // Normalize singles again
  if (single && mantissa != 0)
  {
    if (exponent == 0)
    {
      AdjustExponent(&exponent, &mantissa, &sticky_bit);
      exponent += 0x381;
    }
    else if (exponent == 1)
    {
      exponent += 0x380;
    }
  }

  if (s_debug_prints)
    network_printf(fmt::format("{:01x} {:03x} {:016x}{:016x}\n", sign, exponent, mantissa.upper, mantissa.lower).c_str());

  UnpackedFloat result;
  result.sign = sign;
  result.exponent = exponent;
  result.mantissa = (mantissa >> DOUBLE_FRAC_WIDTH).lower;

  if (result.exponent >= (single ? 0x47f : 0x7ff))
  {
    // Infinity
    if (!forbid_infinity)
    {
      result.exponent = 0x7ff;
      result.mantissa = 0;
    }
    else if (single)
    {
      result.exponent = 0x47e;
      result.mantissa = 0xfffffe0000000;
    }
    else
    {
      result.exponent = 0x7fe;
      result.mantissa = 0xfffffffffffff;
    }
  }

  return PackFloat(result);
}

static void Fmadd(const u64* a, const u64* b, const u64* c, u64* d)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfd %0, 0(%4)\n"
      "lfd %1, 0(%5)\n"
      "lfd %2, 0(%6)\n"
      "fmadd %3, %0, %2, %1\n"
      "stfd %3, 0(%7)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a), "r"(b), "r"(c), "r"(d)
      : "memory"
  );
}

static void Fmsub(const u64* a, const u64* b, const u64* c, u64* d)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfd %0, 0(%4)\n"
      "lfd %1, 0(%5)\n"
      "lfd %2, 0(%6)\n"
      "fmsub %3, %0, %2, %1\n"
      "stfd %3, 0(%7)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a), "r"(b), "r"(c), "r"(d)
      : "memory"
  );
}

static void Fnmadd(const u64* a, const u64* b, const u64* c, u64* d)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfd %0, 0(%4)\n"
      "lfd %1, 0(%5)\n"
      "lfd %2, 0(%6)\n"
      "fnmadd %3, %0, %2, %1\n"
      "stfd %3, 0(%7)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a), "r"(b), "r"(c), "r"(d)
      : "memory"
  );
}

static void Fnmsub(const u64* a, const u64* b, const u64* c, u64* d)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfd %0, 0(%4)\n"
      "lfd %1, 0(%5)\n"
      "lfd %2, 0(%6)\n"
      "fnmsub %3, %0, %2, %1\n"
      "stfd %3, 0(%7)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a), "r"(b), "r"(c), "r"(d)
      : "memory"
  );
}

static void FmaddsDouble(const u64* a, const u64* b, const u64* c, u64* d)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfd %0, 0(%4)\n"
      "lfd %1, 0(%5)\n"
      "lfd %2, 0(%6)\n"
      "fmadds %3, %0, %2, %1\n"
      "stfd %3, 0(%7)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a), "r"(b), "r"(c), "r"(d)
      : "memory"
  );
}

static void FmsubsDouble(const u64* a, const u64* b, const u64* c, u64* d)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfd %0, 0(%4)\n"
      "lfd %1, 0(%5)\n"
      "lfd %2, 0(%6)\n"
      "fmsubs %3, %0, %2, %1\n"
      "stfd %3, 0(%7)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a), "r"(b), "r"(c), "r"(d)
      : "memory"
  );
}

static void FnmaddsDouble(const u64* a, const u64* b, const u64* c, u64* d)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfd %0, 0(%4)\n"
      "lfd %1, 0(%5)\n"
      "lfd %2, 0(%6)\n"
      "fnmadds %3, %0, %2, %1\n"
      "stfd %3, 0(%7)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a), "r"(b), "r"(c), "r"(d)
      : "memory"
  );
}

static void FnmsubsDouble(const u64* a, const u64* b, const u64* c, u64* d)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfd %0, 0(%4)\n"
      "lfd %1, 0(%5)\n"
      "lfd %2, 0(%6)\n"
      "fnmsubs %3, %0, %2, %1\n"
      "stfd %3, 0(%7)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a), "r"(b), "r"(c), "r"(d)
      : "memory"
  );
}

static void FmaddsSingle(const u32* a, const u32* b, const u32* c, u32* d)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfs %0, 0(%4)\n"
      "lfs %1, 0(%5)\n"
      "lfs %2, 0(%6)\n"
      "fmadds %3, %0, %2, %1\n"
      "stfs %3, 0(%7)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a), "r"(b), "r"(c), "r"(d)
      : "memory"
  );
}

static void FmsubsSingle(const u32* a, const u32* b, const u32* c, u32* d)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfs %0, 0(%4)\n"
      "lfs %1, 0(%5)\n"
      "lfs %2, 0(%6)\n"
      "fmsubs %3, %0, %2, %1\n"
      "stfs %3, 0(%7)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a), "r"(b), "r"(c), "r"(d)
      : "memory"
  );
}

static void FnmaddsSingle(const u32* a, const u32* b, const u32* c, u32* d)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfs %0, 0(%4)\n"
      "lfs %1, 0(%5)\n"
      "lfs %2, 0(%6)\n"
      "fnmadds %3, %0, %2, %1\n"
      "stfs %3, 0(%7)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a), "r"(b), "r"(c), "r"(d)
      : "memory"
  );
}

static void FnmsubsSingle(const u32* a, const u32* b, const u32* c, u32* d)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfs %0, 0(%4)\n"
      "lfs %1, 0(%5)\n"
      "lfs %2, 0(%6)\n"
      "fnmsubs %3, %0, %2, %1\n"
      "stfs %3, 0(%7)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a), "r"(b), "r"(c), "r"(d)
      : "memory"
  );
}

static void PsMaddDouble(const u64* a1, const u64* a2, const u64* b1, const u64* b2, const u64* c1,
                         const u64* c2, u64* d1, u64* d2)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfd %0, 0(%5)\n"
      "lfd %1, 0(%7)\n"
      "lfd %2, 0(%9)\n"
      "ps_merge00 %0, %0, %0\n"
      "ps_merge00 %1, %1, %1\n"
      "ps_merge00 %2, %2, %2\n"
      "lfd %0, 0(%4)\n"
      "lfd %1, 0(%6)\n"
      "lfd %2, 0(%8)\n"
      "isync\n"
      "ps_madd %3, %0, %2, %1\n"
      "stfd %3, 0(%10)\n"
      "ps_merge11 %3, %3, %3\n"
      "stfd %3, 0(%11)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a1), "r"(a2), "r"(b1), "r"(b2), "r"(c1), "r"(c2), "r"(d1), "r"(d2)
      : "memory"
  );
}

static void PsMsubDouble(const u64* a1, const u64* a2, const u64* b1, const u64* b2, const u64* c1,
                         const u64* c2, u64* d1, u64* d2)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfd %0, 0(%5)\n"
      "lfd %1, 0(%7)\n"
      "lfd %2, 0(%9)\n"
      "ps_merge00 %0, %0, %0\n"
      "ps_merge00 %1, %1, %1\n"
      "ps_merge00 %2, %2, %2\n"
      "lfd %0, 0(%4)\n"
      "lfd %1, 0(%6)\n"
      "lfd %2, 0(%8)\n"
      "isync\n"
      "ps_msub %3, %0, %2, %1\n"
      "stfd %3, 0(%10)\n"
      "ps_merge11 %3, %3, %3\n"
      "stfd %3, 0(%11)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a1), "r"(a2), "r"(b1), "r"(b2), "r"(c1), "r"(c2), "r"(d1), "r"(d2)
      : "memory"
  );
}

static void PsNmaddDouble(const u64* a1, const u64* a2, const u64* b1, const u64* b2, const u64* c1,
                         const u64* c2, u64* d1, u64* d2)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfd %0, 0(%5)\n"
      "lfd %1, 0(%7)\n"
      "lfd %2, 0(%9)\n"
      "ps_merge00 %0, %0, %0\n"
      "ps_merge00 %1, %1, %1\n"
      "ps_merge00 %2, %2, %2\n"
      "lfd %0, 0(%4)\n"
      "lfd %1, 0(%6)\n"
      "lfd %2, 0(%8)\n"
      "isync\n"
      "ps_nmadd %3, %0, %2, %1\n"
      "stfd %3, 0(%10)\n"
      "ps_merge11 %3, %3, %3\n"
      "stfd %3, 0(%11)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a1), "r"(a2), "r"(b1), "r"(b2), "r"(c1), "r"(c2), "r"(d1), "r"(d2)
      : "memory"
  );
}

static void PsNmsubDouble(const u64* a1, const u64* a2, const u64* b1, const u64* b2, const u64* c1,
                         const u64* c2, u64* d1, u64* d2)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfd %0, 0(%5)\n"
      "lfd %1, 0(%7)\n"
      "lfd %2, 0(%9)\n"
      "ps_merge00 %0, %0, %0\n"
      "ps_merge00 %1, %1, %1\n"
      "ps_merge00 %2, %2, %2\n"
      "lfd %0, 0(%4)\n"
      "lfd %1, 0(%6)\n"
      "lfd %2, 0(%8)\n"
      "isync\n"
      "ps_nmsub %3, %0, %2, %1\n"
      "stfd %3, 0(%10)\n"
      "ps_merge11 %3, %3, %3\n"
      "stfd %3, 0(%11)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a1), "r"(a2), "r"(b1), "r"(b2), "r"(c1), "r"(c2), "r"(d1), "r"(d2)
      : "memory"
  );
}

static void PsMadds0Double(const u64* a1, const u64* a2, const u64* b1, const u64* b2, const u64* c,
                           u64* d1, u64* d2)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfd %0, 0(%5)\n"
      "lfd %1, 0(%7)\n"
      "lfd %2, 0(%9)\n"
      "ps_merge00 %0, %0, %0\n"
      "ps_merge00 %1, %1, %1\n"
      "ps_merge00 %2, %2, %2\n"
      "lfd %0, 0(%4)\n"
      "lfd %1, 0(%6)\n"
      "lfd %2, 0(%8)\n"
      "isync\n"
      "ps_madds0 %3, %0, %2, %1\n"
      "stfd %3, 0(%9)\n"
      "ps_merge11 %3, %3, %3\n"
      "stfd %3, 0(%10)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a1), "r"(a2), "r"(b1), "r"(b2), "r"(c), "r"(d1), "r"(d2)
      : "memory"
  );
}

static void PsMadds1Double(const u64* a1, const u64* a2, const u64* b1, const u64* b2, const u64* c,
                           u64* d1, u64* d2)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfd %0, 0(%5)\n"
      "lfd %1, 0(%7)\n"
      "lfd %2, 0(%8)\n"
      "ps_merge00 %0, %0, %0\n"
      "ps_merge00 %1, %1, %1\n"
      "ps_merge00 %2, %2, %2\n"
      "lfd %0, 0(%4)\n"
      "lfd %1, 0(%6)\n"
      "lfd %2, 0(%10)\n"
      "isync\n"
      "ps_madds1 %3, %0, %2, %1\n"
      "stfd %3, 0(%9)\n"
      "ps_merge11 %3, %3, %3\n"
      "stfd %3, 0(%10)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a1), "r"(a2), "r"(b1), "r"(b2), "r"(c), "r"(d1), "r"(d2)
      : "memory"
  );
}

static void PsMaddSingle(const u32* a1, const u32* a2, const u32* b1, const u32* b2, const u32* c1,
                         const u32* c2, u32* d1, u32* d2)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfs %0, 0(%4)\n"
      "lfs %3, 0(%5)\n"
      "ps_merge00 %0, %0, %3\n"
      "lfs %1, 0(%6)\n"
      "lfs %3, 0(%7)\n"
      "ps_merge00 %1, %1, %3\n"
      "lfs %2, 0(%8)\n"
      "lfs %3, 0(%9)\n"
      "ps_merge00 %2, %2, %3\n"
      "ps_madd %3, %0, %2, %1\n"
      "stfs %3, 0(%10)\n"
      "ps_merge11 %3, %3, %3\n"
      "stfs %3, 0(%11)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a1), "r"(a2), "r"(b1), "r"(b2), "r"(c1), "r"(c2), "r"(d1), "r"(d2)
      : "memory"
  );
}

static void PsMsubSingle(const u32* a1, const u32* a2, const u32* b1, const u32* b2, const u32* c1,
                         const u32* c2, u32* d1, u32* d2)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfs %0, 0(%4)\n"
      "lfs %3, 0(%5)\n"
      "ps_merge00 %0, %0, %3\n"
      "lfs %1, 0(%6)\n"
      "lfs %3, 0(%7)\n"
      "ps_merge00 %1, %1, %3\n"
      "lfs %2, 0(%8)\n"
      "lfs %3, 0(%9)\n"
      "ps_merge00 %2, %2, %3\n"
      "ps_msub %3, %0, %2, %1\n"
      "stfs %3, 0(%10)\n"
      "ps_merge11 %3, %3, %3\n"
      "stfs %3, 0(%11)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a1), "r"(a2), "r"(b1), "r"(b2), "r"(c1), "r"(c2), "r"(d1), "r"(d2)
      : "memory"
  );
}

static void PsNmaddSingle(const u32* a1, const u32* a2, const u32* b1, const u32* b2, const u32* c1,
                         const u32* c2, u32* d1, u32* d2)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfs %0, 0(%4)\n"
      "lfs %3, 0(%5)\n"
      "ps_merge00 %0, %0, %3\n"
      "lfs %1, 0(%6)\n"
      "lfs %3, 0(%7)\n"
      "ps_merge00 %1, %1, %3\n"
      "lfs %2, 0(%8)\n"
      "lfs %3, 0(%9)\n"
      "ps_merge00 %2, %2, %3\n"
      "ps_nmadd %3, %0, %2, %1\n"
      "stfs %3, 0(%10)\n"
      "ps_merge11 %3, %3, %3\n"
      "stfs %3, 0(%11)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a1), "r"(a2), "r"(b1), "r"(b2), "r"(c1), "r"(c2), "r"(d1), "r"(d2)
      : "memory"
  );
}

static void PsNmsubSingle(const u32* a1, const u32* a2, const u32* b1, const u32* b2, const u32* c1,
                         const u32* c2, u32* d1, u32* d2)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfs %0, 0(%4)\n"
      "lfs %3, 0(%5)\n"
      "ps_merge00 %0, %0, %3\n"
      "lfs %1, 0(%6)\n"
      "lfs %3, 0(%7)\n"
      "ps_merge00 %1, %1, %3\n"
      "lfs %2, 0(%8)\n"
      "lfs %3, 0(%9)\n"
      "ps_merge00 %2, %2, %3\n"
      "ps_nmsub %3, %0, %2, %1\n"
      "stfs %3, 0(%10)\n"
      "ps_merge11 %3, %3, %3\n"
      "stfs %3, 0(%11)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a1), "r"(a2), "r"(b1), "r"(b2), "r"(c1), "r"(c2), "r"(d1), "r"(d2)
      : "memory"
  );
}

static void PsMadds0Single(const u32* a1, const u32* a2, const u32* b1, const u32* b2, const u32* c,
                           u32* d1, u32* d2)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfs %0, 0(%4)\n"
      "lfs %3, 0(%5)\n"
      "ps_merge00 %0, %0, %3\n"
      "lfs %1, 0(%6)\n"
      "lfs %3, 0(%7)\n"
      "ps_merge00 %1, %1, %3\n"
      "lfs %2, 0(%8)\n"
      "lfs %3, 0(%9)\n"
      "ps_merge00 %2, %2, %3\n"
      "ps_madds0 %3, %0, %2, %1\n"
      "stfs %3, 0(%9)\n"
      "ps_merge11 %3, %3, %3\n"
      "stfs %3, 0(%10)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a1), "r"(a2), "r"(b1), "r"(b2), "r"(c), "r"(d1), "r"(d2)
      : "memory"
  );
}

static void PsMadds1Single(const u32* a1, const u32* a2, const u32* b1, const u32* b2, const u32* c,
                           u32* d1, u32* d2)
{
  double temp1, temp2, temp3, temp4;
  asm volatile ("lfs %0, 0(%4)\n"
      "lfs %3, 0(%5)\n"
      "ps_merge00 %0, %0, %3\n"
      "lfs %1, 0(%6)\n"
      "lfs %3, 0(%7)\n"
      "ps_merge00 %1, %1, %3\n"
      "lfs %2, 0(%9)\n"
      "lfs %3, 0(%8)\n"
      "ps_merge00 %2, %2, %3\n"
      "ps_madds1 %3, %0, %2, %1\n"
      "stfs %3, 0(%9)\n"
      "ps_merge11 %3, %3, %3\n"
      "stfs %3, 0(%10)\n"
      : "=f"(temp1), "=f"(temp2), "=f"(temp3), "=f"(temp4)
      : "r"(a1), "r"(a2), "r"(b1), "r"(b2), "r"(c), "r"(d1), "r"(d2)
      : "memory"
  );
}

struct TestCase
{
  // Double-precision floating point operands for the calculation a * c + b
  u64 a;
  u64 b;
  u64 c;
};

constexpr std::array<TestCase, 166> TEST_CASES = {{
    // Some simple cases
    {0x4000000000000000, 0x3ff0000000000000, 0x4008000000000000},
    {0x3fe0000000000000, 0x0000000000000000, 0x3fe8000000000000},
    {0x0000000000000000, 0x3a46800000000000, 0x0000000000000000},
    {0x3ff0000000000000, 0xbff0000000000000, 0x3ff0000000000000},
    {0x3ff0000000000000, 0x0000000000000000, 0x0000000000000000},
    {0x3ff0000000000000, 0xc024000000000000, 0x3ff0000000000000},
    {0xbff0000000000000, 0x4024000000000000, 0x3ff0000000000000},
    {0x4020000000000000, 0xc010000000000000, 0x4000000000000000},
    {0xbff0000000000000, 0x3ff0000000000000, 0x3ff0000000000000},
    {0x3ff0000000000000, 0xbff0000000000000, 0x3ff0000000000000},
    // NaN
    {0x7ff00001ffffffff, 0x7ff00002ffffffff, 0x7ff00003ffffffff},
    {0x3ff0000000000000, 0x7ff00002ffffffff, 0x7ff00003ffffffff},
    {0x7ff00001ffffffff, 0x3fe0000000000000, 0x7ff00003ffffffff},
    {0x7ff00001ffffffff, 0x7ff00002ffffffff, 0x3fd0000000000000},
    {0x3ff0000000000000, 0x3fe0000000000000, 0x7ff00003ffffffff},
    {0x3ff0000000000000, 0x7ff00002ffffffff, 0x3fd0000000000000},
    {0x7ff00001ffffffff, 0x3fe0000000000000, 0x3fd0000000000000},
    // Infinity
    {0x7ff0000000000000, 0x3ff0000000000000, 0x0000000000000000},
    {0xfff0000000000000, 0x3ff0000000000000, 0x0000000000000000},
    {0x7ff0000000000000, 0x0000000000000000, 0x3ff0000000000000},
    {0xfff0000000000000, 0x0000000000000000, 0x3ff0000000000000},
    {0x7ff0000000000000, 0x3ff0000000000000, 0x4008000000000000},
    {0x4000000000000000, 0x7ff0000000000000, 0x4008000000000000},
    {0x4000000000000000, 0x3ff0000000000000, 0x7ff0000000000000},
    {0xfff0000000000000, 0x3ff0000000000000, 0x4008000000000000},
    {0x4000000000000000, 0xfff0000000000000, 0x4008000000000000},
    {0x4000000000000000, 0x3ff0000000000000, 0xfff0000000000000},
    {0x7ff0000000000000, 0x3ff0000000000000, 0x7ff0000000000000},
    {0xfff0000000000000, 0x3ff0000000000000, 0x7ff0000000000000},
    {0x7ff0000000000000, 0x3ff0000000000000, 0xfff0000000000000},
    {0xfff0000000000000, 0x3ff0000000000000, 0xfff0000000000000},
    {0x7ff0000000000000, 0x7ff0000000000000, 0x7ff0000000000000},
    {0x7ff0000000000000, 0xfff0000000000000, 0x7ff0000000000000},
    {0xfff0000000000000, 0xfff0000000000000, 0xfff0000000000000},
    // Infinity (double)
    {0x7fe0000000000000, 0x3ff0000000000000, 0x4001000000000000},
    {0x7fe0000000000000, 0xffe0000000000000, 0x4001000000000000},
    // Infinity (single)
    {0x47e0000000000000, 0x3ff0000000000000, 0x4001000000000000},
    {0x47e0000000000000, 0xc7e0000000000000, 0x4001000000000000},
    // Signed zeroes
    {0x0000000000000000, 0x0000000000000000, 0x0000000000000000},
    {0x8000000000000000, 0x0000000000000000, 0x0000000000000000},
    {0x0000000000000000, 0x8000000000000000, 0x0000000000000000},
    {0x0000000000000000, 0x0000000000000000, 0x8000000000000000},
    {0x8000000000000000, 0x8000000000000000, 0x0000000000000000},
    {0x8000000000000000, 0x0000000000000000, 0x8000000000000000},
    {0x0000000000000000, 0x8000000000000000, 0x8000000000000000},
    {0x8000000000000000, 0x8000000000000000, 0x8000000000000000},
    // Cases where precision matters (double)
    {0x3ff0000000000000, 0x3c90000000000000, 0x3ff0000000000000},
    {0x3ff0000000000000, 0x3ca0000000000000, 0x3ff0000000000000},
    {0x3ff0000000000000, 0x3cb0000000000000, 0x3ff0000000000000},
    {0x3ff0000000000001, 0x3c90000000000000, 0x3ff0000000000000},
    {0x3ff0000000000001, 0x3ca0000000000000, 0x3ff0000000000000},
    {0x3ff0000000000001, 0x3cb0000000000000, 0x3ff0000000000000},
    {0x3fffffffffffffff, 0x3cc0000000000000, 0x3fffffffffffffff},
    {0x3fffffffffffffff, 0x3cd0000000000000, 0x3fffffffffffffff},
    {0x3ff0000000000001, 0x0000000000000000, 0x4008000000000000},
    // Cases where precision matters (single)
    {0x3ff0000000000000, 0x3e60000000000000, 0x3ff0000000000000},
    {0x3ff0000000000000, 0x3e70000000000000, 0x3ff0000000000000},
    {0x3ff0000000000000, 0x3e80000000000000, 0x3ff0000000000000},
    {0x3ff0000020000000, 0x3e60000000000000, 0x3ff0000000000000},
    {0x3ff0000020000000, 0x3e70000000000000, 0x3ff0000000000000},
    {0x3ff0000020000000, 0x3e80000000000000, 0x3ff0000000000000},
    {0x3fffffffe0000000, 0x3e90000000000000, 0x3fffffffe0000000},
    {0x3fffffffe0000000, 0x3ea0000000000000, 0x3fffffffe0000000},
    {0x3ff0000020000000, 0x0000000000000000, 0x4008000000000000},
    // Cases that are sensitive to intermediate rounding (double)
    {0x3ff0000000000004, 0x3c90000000000000, 0x3ff2000000000000},
    {0xbff0000000000004, 0xbc90000000000000, 0x3ff2000000000000},
    {0x3ff000000000000a, 0x3c90000000000000, 0x3ff2000000000000},
    {0xbff000000000000a, 0xbc90000000000000, 0x3ff2000000000000},
    {0x3ff000000000000c, 0xbc90000000000000, 0x3ff2000000000000},
    {0xbff000000000000c, 0x3c90000000000000, 0x3ff2000000000000},
    {0x3ff0000000000006, 0xbc90000000000000, 0x3ff2000000000000},
    {0xbff0000000000006, 0x3c90000000000000, 0x3ff2000000000000},
    {0x3ff0000000000001, 0xbff0000020000001, 0x3ff0000020000000},
    {0xbff0000000000001, 0x3ff0000020000001, 0x3ff0000020000000},
    // Cases that are sensitive to intermediate rounding (single)
    {0x3ff0000080000000, 0x3e60000000000000, 0x3ff2000000000000},
    {0xbff0000080000000, 0xbe60000000000000, 0x3ff2000000000000},
    {0x3ff0000140000000, 0x3e60000000000000, 0x3ff2000000000000},
    {0xbff0000140000000, 0xbe60000000000000, 0x3ff2000000000000},
    {0x3ff0000180000000, 0xbe60000000000000, 0x3ff2000000000000},
    {0xbff0000180000000, 0x3e60000000000000, 0x3ff2000000000000},
    {0x3ff00000c0000000, 0xbe60000000000000, 0x3ff2000000000000},
    {0xbff00000c0000000, 0x3e60000000000000, 0x3ff2000000000000},
    {0x3ff0000020000000, 0xbff0000040000000, 0x3ff0000020000000},
    {0xbff0000020000000, 0x3ff0000040000000, 0x3ff0000020000000},
    // Cases that are sensitive to double rounding
    {0x3ff0000010000000, 0x3ca0000000000000, 0x3ff0000000000000},
    {0xbff0000010000000, 0xbca0000000000000, 0x3ff0000000000000},
    {0x3ff000002fffffff, 0x3ca0000000000000, 0x3ff0000000000000},
    {0xbff000002fffffff, 0xbca0000000000000, 0x3ff0000000000000},
    {0x3ff0000030000000, 0xbca0000000000000, 0x3ff0000000000000},
    {0xbff0000030000000, 0x3ca0000000000000, 0x3ff0000000000000},
    {0x3ff0000010000001, 0xbca0000000000000, 0x3ff0000000000000},
    {0xbff0000010000001, 0x3ca0000000000000, 0x3ff0000000000000},
    {0x3ff0000080000000, 0x3ca0000000000000, 0x3ff2000000000000},
    {0xbff0000080000000, 0xbca0000000000000, 0x3ff2000000000000},
    {0x3ff0000180000000, 0xbca0000000000000, 0x3ff2000000000000},
    {0xbff0000180000000, 0x3ca0000000000000, 0x3ff2000000000000},
    {0x3ff555556aaaaaab, 0x0000000000000000, 0x3ff8000000000000},
    {0xbff555556aaaaaab, 0x0000000000000000, 0x3ff8000000000000},
    {0x3ff5555595555555, 0x0000000000000000, 0x3ff8000000000000},
    {0xbff5555595555555, 0x0000000000000000, 0x3ff8000000000000},
    // Cases that are sensitive to double rounding, with denormals
    {0x3ff0000010000000, 0x0000000000000001, 0x3ff0000000000000},
    {0xbff0000010000000, 0x8000000000000001, 0x3ff0000000000000},
    {0x3ff000002fffffff, 0x0000000000000001, 0x3ff0000000000000},
    {0xbff000002fffffff, 0x8000000000000001, 0x3ff0000000000000},
    {0x3ff0000030000000, 0x8000000000000001, 0x3ff0000000000000},
    {0xbff0000030000000, 0x0000000000000001, 0x3ff0000000000000},
    {0x3ff0000010000001, 0x8000000000000001, 0x3ff0000000000000},
    {0xbff0000010000001, 0x0000000000000001, 0x3ff0000000000000},
    // Cases that are sensitive to double rounding, with terms smaller than denormals
    {0x0010000000000000, 0x3ff0000010000000, 0x3810000000000000},
    {0x8010000000000000, 0xbff0000010000000, 0x3810000000000000},
    {0x0010000000000000, 0x3ff000002fffffff, 0x3810000000000000},
    {0x8010000000000000, 0xbff000002fffffff, 0x3810000000000000},
    {0x8010000000000000, 0x3ff0000030000000, 0x3810000000000000},
    {0x0010000000000000, 0xbff0000030000000, 0x3810000000000000},
    {0x8010000000000000, 0x3ff0000010000001, 0x3810000000000000},
    {0x0010000000000000, 0xbff0000010000001, 0x3810000000000000},
    // Denormals (double)
    {0x0000000000000000, 0x0000000000000001, 0x0000000000000000},
    {0x0000000000000002, 0x0000000000000001, 0x0000000000000003},
    {0x0000000000000001, 0x8000000000000001, 0x0000000000000001},
    {0x0000000000000008, 0x8000000000000004, 0x4000000000000000},
    {0x0010000000000000, 0x0008000000004321, 0x3fe0000000000000},
    {0x0008000000000000, 0x8008000000004321, 0x4000000000000000},
    {0x3ff0000000000000, 0x0000000000000001, 0x3ff0000000000000},
    {0x3ff0000000000000, 0x8000000000000001, 0x3ff0000000000000},
    {0x001fffffffffffff, 0x0000000000000000, 0x3fe0000000000000},
    {0x7ff0000000000000, 0x3ff0000000000000, 0x0000000000000001},
    // Denormals (single)
    {0x0000000000000000, 0x36a0000000000000, 0x0000000000000000},
    {0x36b0000000000000, 0x36a0000000000000, 0x36b8000000000000},
    {0x36a0000000000000, 0xb6a0000000000000, 0x36a0000000000000},
    {0x36d0000000000000, 0xb6c0000000000000, 0x4000000000000000},
    {0x3810000000000000, 0x380010c840000000, 0x3fe0000000000000},
    {0x3800000000000000, 0xb80010c840000000, 0x4000000000000000},
    {0x3ff0000000000000, 0x36a0000000000000, 0x3ff0000000000000},
    {0x3ff0000000000000, 0xb6a0000000000000, 0x3ff0000000000000},
    {0x381fffffe0000000, 0x0000000000000000, 0x3fe0000000000000},
    {0x7ff0000000000000, 0x3ff0000000000000, 0x36a0000000000000},
    // C rounding
    {0x3ff0000000000000, 0xc7dffffe00000000, 0x47dffffffbffffff},
    {0x47dffffffbffffff, 0xc7dffffe00000000, 0x3ff0000000000000},
    {0x3ff0000000000000, 0xc7dffffe00000000, 0x47dffffff7ffffff},
    {0x47dffffff7ffffff, 0xc7dffffe00000000, 0x3ff0000000000000},
    {0x3ff0000000000000, 0xc7dffffe00000000, 0x47dfffffefffffff},
    {0x47dfffffefffffff, 0xc7dffffe00000000, 0x3ff0000000000000},
    {0x0010000000000000, 0x0ff0123400000000, 0x7fdfffffffffffff},
    {0x7fdfffffffffffff, 0x0ff0123400000000, 0x0010000000000000},
    {0x0010000000000000, 0x0ff0123400000000, 0x7fefffffffffffff},
    {0x7fefffffffffffff, 0x0ff0123400000000, 0x0010000000000000},
    // C rounding with denormals (double)
    {0x0001000000000000, 0x0000000000000000, 0x7fdfffffffffffff},
    {0x7fdfffffffffffff, 0x0000000000000000, 0x0001000000000000},
    {0x0000000000000001, 0x0000000000000000, 0x7fdfffffffffffff},
    {0x7fdfffffffffffff, 0x0000000000000000, 0x0000000000000001},
    {0x7fe0000000000000, 0xbff0000000000000, 0x00080000fdffffff},
    {0x00080000fdffffff, 0xbff0000000000000, 0x7fe0000000000000},
    {0x7fe0000000000000, 0xbff0000000000000, 0x00080000fbffffff},
    {0x00080000fbffffff, 0xbff0000000000000, 0x7fe0000000000000},
    {0x7fe0000000000000, 0xbff0000000000000, 0x00080000f7ffffff},
    {0x00080000f7ffffff, 0xbff0000000000000, 0x7fe0000000000000},
    {0x7fe0000000000000, 0xbff0000000000000, 0x00040000feffffff},
    {0x00040000feffffff, 0xbff0000000000000, 0x7fe0000000000000},
    {0x7fe0000000000000, 0xbff0000000000000, 0x00040000fdffffff},
    {0x00040000fdffffff, 0xbff0000000000000, 0x7fe0000000000000},
    {0x7fe0000000000000, 0xbff0000000000000, 0x00040000fbffffff},
    {0x00040000fbffffff, 0xbff0000000000000, 0x7fe0000000000000},
    // C rounding with denormals (single)
    {0x3800000000000000, 0x0000000000000000, 0x7fdfffffffffffff},
    {0x7fdfffffffffffff, 0x0000000000000000, 0x3800000000000000},
    {0x36a0000000000000, 0x0000000000000000, 0x7fdfffffffffffff},
    {0x7fdfffffffffffff, 0x0000000000000000, 0x36a0000000000000},
}};

static u32 ToSingle(u64 value)
{
  return std::bit_cast<u32>(static_cast<float>(std::bit_cast<double>(TruncateMantissaBits(value))));
}

static u64 ToDouble(u32 value)
{
  return std::bit_cast<u64>(static_cast<double>(std::bit_cast<float>(value)));
}

static u32 FlipSignIfNotNaN(u32 value)
{
  if ((value & FLOAT_EXP) == FLOAT_EXP && (value & FLOAT_FRAC) != 0)
    return value;
  else
    return value ^ FLOAT_SIGN;
}

static u64 FlipSignIfNotNaN(u64 value)
{
  if ((value & DOUBLE_EXP) == DOUBLE_EXP && (value & DOUBLE_FRAC) != 0)
    return value;
  else
    return value ^ DOUBLE_SIGN;
}

template <typename F>
static void FmaTest(F hardware_fma, u32 fpscr_low_bits, bool negate_b, bool negate_result, bool single)
{
  START_TEST();

  for (const TestCase& test_case : TEST_CASES)
  {
    const u64 a = test_case.a;
    u64 b = test_case.b;
    const u64 c = test_case.c;

    const u64 expected = SoftwareFma(a, b, c, fpscr_low_bits, single);

    if (negate_b)
      b = FlipSignIfNotNaN(b);

    u64 actual = 0x0123456789abcdef;
    hardware_fma(&a, &b, &c, &actual);

    if (negate_result)
      actual = FlipSignIfNotNaN(actual);

    DO_TEST(actual == expected, "\n"
                                "Input: a={:016x}, b={:016x}, c={:016x}\n"
                                "     got {:016x}\n"
                                "expected {:016x}",
            a, b, c, actual, expected);
  }

  END_TEST();
}

template <typename F>
static void FmaTestSingleInputs(F hardware_fma, bool negate_b, bool negate_result, u32 fpscr_low_bits)
{
  START_TEST();

  for (const TestCase& test_case : TEST_CASES)
  {
    const u32 a = ToSingle(test_case.a);
    u32 b = ToSingle(test_case.b);
    const u32 c = ToSingle(test_case.c);

    const u32 expected = ToSingle(SoftwareFma(ToDouble(a), ToDouble(b), ToDouble(c), fpscr_low_bits, true));

    if (negate_b)
      b = FlipSignIfNotNaN(b);

    u32 actual = 0x01234567;
    hardware_fma(&a, &b, &c, &actual);

    if (negate_result)
      actual = FlipSignIfNotNaN(actual);

    DO_TEST(actual == expected, "\n"
                                "Input: a={:08x}, b={:08x}, c={:08x}\n"
                                "     got {:08x}\n"
                                "expected {:08x}",
            a, b, c, actual, expected);
  }

  END_TEST();
}

template <typename F>
static void FmaTestPaired(F hardware_fma, u32 fpscr_low_bits, bool negate_b, bool negate_result)
{
  START_TEST();

  for (const TestCase& test_case_1 : TEST_CASES)
  {
    for (const TestCase& test_case_2 : TEST_CASES)
    {
      const u64 a1 = test_case_1.a;
      const u64 a2 = TruncateMantissaBits(test_case_2.a);
      u64 b1 = test_case_1.b;
      u64 b2 = TruncateMantissaBits(test_case_2.b);
      const u64 c1 = test_case_1.c;
      const u64 c2 = TruncateMantissaBits(test_case_2.c);

      const u64 expected_1 = SoftwareFma(a1, b1, c1, fpscr_low_bits, true);
      const u64 expected_2 = SoftwareFma(a2, b2, c2, fpscr_low_bits, true);

      if (negate_b)
      {
        b1 = FlipSignIfNotNaN(b1);
        b2 = FlipSignIfNotNaN(b2);
      }

      u64 actual_1 = 0x0123456789abcdef;
      u64 actual_2 = 0x0123456789abcdef;
      hardware_fma(&a1, &a2, &b1, &b2, &c1, &c2, &actual_1, &actual_2);

      if (negate_result)
      {
        actual_1 = FlipSignIfNotNaN(actual_1);
        actual_2 = FlipSignIfNotNaN(actual_2);
      }

      DO_TEST(actual_1 == expected_1 && actual_2 == expected_2, "\n"
              "Input: a={:016x} {:016x}, b={:016x} {:016x}, c={:016x} {:016x}\n"
              "     got {:016x} {:016x}\n"
              "expected {:016x} {:016x}",
              a1, a2, b1, b2, c1, c2, actual_1, actual_2, expected_1, expected_2);
    }
  }

  END_TEST();
}

template <typename F>
static void FmaTestPairedSingleInputs(F hardware_fma, u32 fpscr_low_bits, bool negate_b, bool negate_result)
{
  START_TEST();

  for (const TestCase& test_case_1 : TEST_CASES)
  {
    for (const TestCase& test_case_2 : TEST_CASES)
    {
      const u32 a1 = ToSingle(test_case_1.a);
      const u32 a2 = ToSingle(test_case_2.a);
      u32 b1 = ToSingle(test_case_1.b);
      u32 b2 = ToSingle(test_case_2.b);
      const u32 c1 = ToSingle(test_case_1.c);
      const u32 c2 = ToSingle(test_case_2.c);

      const u32 expected_1 = ToSingle(SoftwareFma(ToDouble(a1), ToDouble(b1), ToDouble(c1), fpscr_low_bits, true));
      const u32 expected_2 = ToSingle(SoftwareFma(ToDouble(a2), ToDouble(b2), ToDouble(c2), fpscr_low_bits, true));

      if (negate_b)
      {
        b1 = FlipSignIfNotNaN(b1);
        b2 = FlipSignIfNotNaN(b2);
      }

      u32 actual_1 = 0x01234567;
      u32 actual_2 = 0x01234567;
      hardware_fma(&a1, &a2, &b1, &b2, &c1, &c2, &actual_1, &actual_2);

      if (negate_result)
      {
        actual_1 = FlipSignIfNotNaN(actual_1);
        actual_2 = FlipSignIfNotNaN(actual_2);
      }

      DO_TEST(actual_1 == expected_1 && actual_2 == expected_2, "\n"
              "Input: a={:08x} {:08x}, b={:08x} {:08x}, c={:08x} {:08x}\n"
              "     got {:08x} {:08x}\n"
              "expected {:08x} {:08x}",
              a1, a2, b1, b2, c1, c2, actual_1, actual_2, expected_1, expected_2);
    }
  }

  END_TEST();
}

static void FmaTestPaired0(u32 fpscr_low_bits)
{
  START_TEST();

  for (const TestCase& test_case_1 : TEST_CASES)
  {
    for (const TestCase& test_case_2 : TEST_CASES)
    {
      const u64 a1 = test_case_1.a;
      const u64 a2 = TruncateMantissaBits(test_case_2.a);
      const u64 b1 = test_case_1.b;
      const u64 b2 = TruncateMantissaBits(test_case_2.b);
      // TODO: If we don't truncate c's mantissa bits, the ps0 calculation behaves as expected,
      // but the ps1 calculation behaves in a way that matches neither c being truncated nor c
      // not being truncated. This could use further investigation.
      const u64 c = TruncateMantissaBits(test_case_1.c);

      const u64 expected_1 = SoftwareFma(a1, b1, c, fpscr_low_bits, true);
      const u64 expected_2 = SoftwareFma(a2, b2, c, fpscr_low_bits, true);

      u64 actual_1 = 0x0123456789abcdef;
      u64 actual_2 = 0x0123456789abcdef;
      PsMadds0Double(&a1, &a2, &b1, &b2, &c, &actual_1, &actual_2);

      DO_TEST(actual_1 == expected_1 && actual_2 == expected_2, "\n"
              "Input: a={:016x} {:016x}, b={:016x} {:016x}, c={:016x}\n"
              "     got {:016x} {:016x}\n"
              "expected {:016x} {:016x}",
              a1, a2, b1, b2, c, actual_1, actual_2, expected_1, expected_2);
    }
  }

  END_TEST();
}

static void FmaTestPaired1(u32 fpscr_low_bits)
{
  START_TEST();

  for (const TestCase& test_case_1 : TEST_CASES)
  {
    for (const TestCase& test_case_2 : TEST_CASES)
    {
      const u64 a1 = test_case_1.a;
      const u64 a2 = TruncateMantissaBits(test_case_2.a);
      const u64 b1 = test_case_1.b;
      const u64 b2 = TruncateMantissaBits(test_case_2.b);
      const u64 c = TruncateMantissaBits(test_case_2.c);

      const u64 expected_1 = SoftwareFma(a1, b1, c, fpscr_low_bits, true);
      const u64 expected_2 = SoftwareFma(a2, b2, c, fpscr_low_bits, true);

      u64 actual_1 = 0x0123456789abcdef;
      u64 actual_2 = 0x0123456789abcdef;
      PsMadds1Double(&a1, &a2, &b1, &b2, &c, &actual_1, &actual_2);

      DO_TEST(actual_1 == expected_1 && actual_2 == expected_2, "\n"
              "Input: a={:016x} {:016x}, b={:016x} {:016x}, c={:016x}\n"
              "     got {:016x} {:016x}\n"
              "expected {:016x} {:016x}",
              a1, a2, b1, b2, c, actual_1, actual_2, expected_1, expected_2);
    }
  }

  END_TEST();
}

static void FmaTestPaired0SingleInputs(u32 fpscr_low_bits)
{
  START_TEST();

  for (const TestCase& test_case_1 : TEST_CASES)
  {
    for (const TestCase& test_case_2 : TEST_CASES)
    {
      const u32 a1 = ToSingle(test_case_1.a);
      const u32 a2 = ToSingle(test_case_2.a);
      const u32 b1 = ToSingle(test_case_1.b);
      const u32 b2 = ToSingle(test_case_2.b);
      const u32 c = ToSingle(test_case_1.c);

      const u32 expected_1 = ToSingle(SoftwareFma(ToDouble(a1), ToDouble(b1), ToDouble(c), fpscr_low_bits, true));
      const u32 expected_2 = ToSingle(SoftwareFma(ToDouble(a2), ToDouble(b2), ToDouble(c), fpscr_low_bits, true));

      u32 actual_1 = 0x01234567;
      u32 actual_2 = 0x01234567;
      PsMadds0Single(&a1, &a2, &b1, &b2, &c, &actual_1, &actual_2);

      DO_TEST(actual_1 == expected_1 && actual_2 == expected_2, "\n"
              "Input: a={:08x} {:08x}, b={:08x} {:08x}, c={:08x}\n"
              "     got {:08x} {:08x}\n"
              "expected {:08x} {:08x}",
              a1, a2, b1, b2, c, actual_1, actual_2, expected_1, expected_2);
    }
  }

  END_TEST();
}

static void FmaTestPaired1SingleInputs(u32 fpscr_low_bits)
{
  START_TEST();

  for (const TestCase& test_case_1 : TEST_CASES)
  {
    for (const TestCase& test_case_2 : TEST_CASES)
    {
      const u32 a1 = ToSingle(test_case_1.a);
      const u32 a2 = ToSingle(test_case_2.a);
      const u32 b1 = ToSingle(test_case_1.b);
      const u32 b2 = ToSingle(test_case_2.b);
      const u32 c = ToSingle(test_case_2.c);

      const u32 expected_1 = ToSingle(SoftwareFma(ToDouble(a1), ToDouble(b1), ToDouble(c), fpscr_low_bits, true));
      const u32 expected_2 = ToSingle(SoftwareFma(ToDouble(a2), ToDouble(b2), ToDouble(c), fpscr_low_bits, true));

      u32 actual_1 = 0x01234567;
      u32 actual_2 = 0x01234567;
      PsMadds1Single(&a1, &a2, &b1, &b2, &c, &actual_1, &actual_2);

      DO_TEST(actual_1 == expected_1 && actual_2 == expected_2, "\n"
              "Input: a={:08x} {:08x}, b={:08x} {:08x}, c={:08x}\n"
              "     got {:08x} {:08x}\n"
              "expected {:08x} {:08x}",
              a1, a2, b1, b2, c, actual_1, actual_2, expected_1, expected_2);
    }
  }

  END_TEST();
}

int main()
{
  network_init();
  WPAD_Init();

  for (u32 fpscr_low_bits = 0; fpscr_low_bits < 8; ++fpscr_low_bits)
  {
    asm volatile("mtfsf 7, %0" ::"f"(static_cast<u64>(fpscr_low_bits)));
    const std::string parameters =
            fmt::format("NI={}, RN={}", u32(GetNI(fpscr_low_bits)), GetRN(fpscr_low_bits));

    network_printf(fmt::format("Testing fmadd with {}...\n", parameters).c_str());
    FmaTest(&Fmadd, fpscr_low_bits, false, false, false);
    network_printf(fmt::format("Testing fmsub with {}...\n", parameters).c_str());
    FmaTest(&Fmsub, fpscr_low_bits, true, false, false);
    network_printf(fmt::format("Testing fnmadd with {}...\n", parameters).c_str());
    FmaTest(&Fnmadd, fpscr_low_bits, false, true, false);
    network_printf(fmt::format("Testing fnmsub with {}...\n", parameters).c_str());
    FmaTest(&Fnmsub, fpscr_low_bits, true, true, false);

    network_printf(fmt::format("Testing fmadds (double inputs) with {}...\n", parameters).c_str());
    FmaTest(&FmaddsDouble, fpscr_low_bits, false, false, true);
    network_printf(fmt::format("Testing fmsubs (double inputs) with {}...\n", parameters).c_str());
    FmaTest(&FmsubsDouble, fpscr_low_bits, true, false, true);
    network_printf(fmt::format("Testing fnmadds (double inputs) with {}...\n", parameters).c_str());
    FmaTest(&FnmaddsDouble, fpscr_low_bits, false, true, true);
    network_printf(fmt::format("Testing fnmsubs (double inputs) with {}...\n", parameters).c_str());
    FmaTest(&FnmsubsDouble, fpscr_low_bits, true, true, true);

    network_printf(fmt::format("Testing fmadds (single inputs) with {}...\n", parameters).c_str());
    FmaTestSingleInputs(&FmaddsSingle, false, false, fpscr_low_bits);
    network_printf(fmt::format("Testing fmsubs (single inputs) with {}...\n", parameters).c_str());
    FmaTestSingleInputs(&FmsubsSingle, true, false, fpscr_low_bits);
    network_printf(fmt::format("Testing fnmadds (single inputs) with {}...\n", parameters).c_str());
    FmaTestSingleInputs(&FnmaddsSingle, false, true, fpscr_low_bits);
    network_printf(fmt::format("Testing fnmsubs (single inputs) with {}...\n", parameters).c_str());
    FmaTestSingleInputs(&FnmsubsSingle, true, true, fpscr_low_bits);

    network_printf(fmt::format("Testing ps_madd (double inputs) with {}...\n", parameters).c_str());
    FmaTestPaired(&PsMaddDouble, fpscr_low_bits, false, false);
    network_printf(fmt::format("Testing ps_msub (double inputs) with {}...\n", parameters).c_str());
    FmaTestPaired(&PsMsubDouble, fpscr_low_bits, true, false);
    network_printf(fmt::format("Testing ps_nmadd (double inputs) with {}...\n", parameters).c_str());
    FmaTestPaired(&PsNmaddDouble, fpscr_low_bits, false, true);
    network_printf(fmt::format("Testing ps_nmsub (double inputs) with {}...\n", parameters).c_str());
    FmaTestPaired(&PsNmsubDouble, fpscr_low_bits, true, true);

    network_printf(fmt::format("Testing ps_madds0 (double inputs) with {}...\n", parameters).c_str());
    FmaTestPaired0(fpscr_low_bits);
    network_printf(fmt::format("Testing ps_madds1 (double inputs) with {}...\n", parameters).c_str());
    FmaTestPaired1(fpscr_low_bits);

    network_printf(fmt::format("Testing ps_madd (single inputs) with {}...\n", parameters).c_str());
    FmaTestPairedSingleInputs(&PsMaddSingle, fpscr_low_bits, false, false);
    network_printf(fmt::format("Testing ps_msub (single inputs) with {}...\n", parameters).c_str());
    FmaTestPairedSingleInputs(&PsMsubSingle, fpscr_low_bits, true, false);
    network_printf(fmt::format("Testing ps_nmadd (single inputs) with {}...\n", parameters).c_str());
    FmaTestPairedSingleInputs(&PsNmaddSingle, fpscr_low_bits, false, true);
    network_printf(fmt::format("Testing ps_nmsub (single inputs) with {}...\n", parameters).c_str());
    FmaTestPairedSingleInputs(&PsNmsubSingle, fpscr_low_bits, true, true);

    network_printf(fmt::format("Testing ps_madds0 (single inputs) with {}...\n", parameters).c_str());
    FmaTestPaired0SingleInputs(fpscr_low_bits);
    network_printf(fmt::format("Testing ps_madds1 (single inputs) with {}...\n", parameters).c_str());
    FmaTestPaired1SingleInputs(fpscr_low_bits);
  }

  network_printf("Shutting down...\n");
  network_shutdown();

  return 0;
}
