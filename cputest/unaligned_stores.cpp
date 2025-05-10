#include <algorithm>
#include <array>
#include <cinttypes>

#include <gctypes.h>
#include <ogc/irq.h>
#include <ogc/system.h>
#include <wiiuse/wpad.h>

#include "Common/Align.h"
#include "Common/BitUtils.h"
#include "Common/hwtests.h"

// This test covers the behavior described in https://bugs.dolphin-emu.org/issues/12565

enum class Mode
{
  Regular,
  Swap,
  Stswi,
  Stswx,
};

static constexpr std::array<const char*, 4> MODE_NAMES = {
  "regular",
  "swap",
  "stswi",
  "stswx",
};

static constexpr std::array<u8, 32> STSW_VALUES = {
  0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
  0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
  0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
  0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
};

static constexpr uintptr_t PAGE_SIZE = 4096;
static constexpr uintptr_t CACHE_LINE_SIZE = 32;

static volatile u32* const s_pi_reg = reinterpret_cast<volatile u32*>(0xCC003000);

static volatile bool s_pi_error_occurred = false;

static u32 Read(volatile u8* ptr)
{
  return *reinterpret_cast<volatile u32*>(ptr);
}

static void Write(volatile u8* ptr, u32 value, u32 size, bool swap)
{
  switch (size)
  {
  case 4:
    if (swap)
      asm volatile("stwbrx %0, 0, %1" :: "r"(value), "r"(ptr));
    else
      asm volatile("stwx %0, 0, %1" :: "r"(value), "r"(ptr));
    break;
  case 2:
    if (swap)
      asm volatile("sthbrx %0, 0, %1" :: "r"(value), "r"(ptr));
    else
      asm volatile("sthx %0, 0, %1" :: "r"(value), "r"(ptr));
    break;
  case 1:
    asm volatile("stbx %0, 0, %1" :: "r"(value), "r"(ptr));
    break;
  }
}

static void WriteWithSimulatedQuirks(uintptr_t alignment, volatile u8* ptr, u32 value, u32 size)
{
  const uintptr_t misalignment_32 = alignment & 3;

  if (misalignment_32 || size < 4)
  {
    const uintptr_t misalignment_64 = alignment & 7;
    const uintptr_t count = misalignment_64 + size;

    value = Common::RotateRight(value, (misalignment_32 + size) * 8);

    for (uintptr_t i = 0; i < count; i += 8)
    {
      Write(ptr - misalignment_64 + i, value, sizeof(u32), false);
      Write(ptr - misalignment_64 + i + 4, value, sizeof(u32), false);
    }
  }
  else
  {
    Write(ptr, value, size, false);
  }
}

template <u32 OutSize>
static void WriteStswiImpl(volatile u8* out_ptr, const u8* in_ptr, u32 in_size)
{
  asm volatile("mtxer %3\n"
               "lswx 24, 0, %1\n"
               "stswi 24, %0, %2\n"
               :: "r"(out_ptr), "r"(in_ptr), "i"(OutSize), "r"(in_size)
               : "r24", "r25", "r26", "r27", "r28", "r29", "r30", "r31");
}

static void WriteStswi(volatile u8* out_ptr, const u8* in_ptr, u32 out_size, u32 in_size)
{
  // stswi supports sizes from 1 to 32, with 32 being encoded as 0
  static constexpr std::array stswi_table =
  {
    &WriteStswiImpl<0>,  &WriteStswiImpl<1>,  &WriteStswiImpl<2>,  &WriteStswiImpl<3>,
    &WriteStswiImpl<4>,  &WriteStswiImpl<5>,  &WriteStswiImpl<6>,  &WriteStswiImpl<7>,
    &WriteStswiImpl<8>,  &WriteStswiImpl<9>,  &WriteStswiImpl<10>, &WriteStswiImpl<11>,
    &WriteStswiImpl<12>, &WriteStswiImpl<13>, &WriteStswiImpl<14>, &WriteStswiImpl<15>,
    &WriteStswiImpl<16>, &WriteStswiImpl<17>, &WriteStswiImpl<18>, &WriteStswiImpl<19>,
    &WriteStswiImpl<20>, &WriteStswiImpl<21>, &WriteStswiImpl<22>, &WriteStswiImpl<23>,
    &WriteStswiImpl<24>, &WriteStswiImpl<25>, &WriteStswiImpl<26>, &WriteStswiImpl<27>,
    &WriteStswiImpl<28>, &WriteStswiImpl<29>, &WriteStswiImpl<30>, &WriteStswiImpl<31>,
  };

  stswi_table[out_size & 31](out_ptr, in_ptr, in_size);
}

static void WriteStswx(volatile u8* out_ptr, const u8* in_ptr, u32 out_size, u32 in_size)
{
  asm volatile("mtxer %3\n"
               "lswx 24, 0, %1\n"
               "mtxer %2\n"
               "stswx 24, 0, %0\n"
               :: "r"(out_ptr), "r"(in_ptr), "r"(out_size), "r"(in_size)
               : "r24", "r25", "r26", "r27", "r28", "r29", "r30", "r31");
}

static void WriteStswSimulated(volatile u8* out_ptr, const u8* in_ptr, u32 size)
{
  std::copy(in_ptr, in_ptr + size, out_ptr);
}

static void UnalignedStoresTest(volatile u8* ptr, u32 size, bool cached_memory, Mode mode)
{
  network_printf("Starting test using ptr 0x%" PRIxPTR ", size %u, mode %s\n",
                 reinterpret_cast<uintptr_t>(ptr), size, MODE_NAMES[static_cast<u32>(mode)]);

  volatile u8 reference_buffer[32];
  const u32 word = 0x12345678;
  const u32 swapped_word = (mode == Mode::Swap) ? (size == 2 ? 0x12347856 : 0x78563412) : word;
  const u32 fill_word = 0x01020304;
  const bool stsw = mode == Mode::Stswi || mode == Mode::Stswx;

  for (size_t i = 0; i <= 32 - size; ++i)
  {
    // Use 32-bit writes to avoid accidentally triggering the behavior we're trying to test
    std::fill(reinterpret_cast<volatile u32*>(ptr), reinterpret_cast<volatile u32*>(ptr + 32),
              fill_word);
    std::fill(reinterpret_cast<volatile u32*>(reference_buffer),
              reinterpret_cast<volatile u32*>(reference_buffer + 32), fill_word);

    s_pi_error_occurred = false;

    if (stsw)
    {
      // The actual write that we are testing the behavior of
      if (mode == Mode::Stswi)
        WriteStswi(ptr + i, STSW_VALUES.data(), size, STSW_VALUES.size());
      else
        WriteStswx(ptr + i, STSW_VALUES.data(), size, STSW_VALUES.size());

      WriteStswSimulated(reference_buffer + i, STSW_VALUES.data(), size);
    }
    else
    {
      // The actual write that we are testing the behavior of
      Write(ptr + i, word, size, mode == Mode::Swap);

      if (cached_memory)
        Write(reference_buffer + i, swapped_word, size, false);
      else
        WriteWithSimulatedQuirks(i, reference_buffer + i, swapped_word, size);
    }

    DO_TEST(std::equal(ptr, ptr + 32, reference_buffer),
            "{}-byte write to {} failed\n"
            "ACTUAL:              EXPECTED:\n"
            "{:08X} {:08X}    {:08X} {:08X}\n"
            "{:08X} {:08X}    {:08X} {:08X}\n"
            "{:08X} {:08X}    {:08X} {:08X}\n"
            "{:08X} {:08X}    {:08X} {:08X}\n",
            size, fmt::ptr(ptr + i),
            Read(ptr), Read(ptr + 4), Read(reference_buffer), Read(reference_buffer + 4),
            Read(ptr + 8), Read(ptr + 12), Read(reference_buffer + 8), Read(reference_buffer + 12),
            Read(ptr + 16), Read(ptr + 20), Read(reference_buffer + 16), Read(reference_buffer + 20),
            Read(ptr + 24), Read(ptr + 28), Read(reference_buffer + 24), Read(reference_buffer + 28));

    // TODO: Why does this trigger even for aligned 32-bit writes? Might be a bug in the test code
    int pi_error_expected = !cached_memory && !stsw /* && ((i & 3) || (size & 3)) */;
    int pi_error_actual = s_pi_error_occurred;

    DO_TEST(pi_error_actual == pi_error_expected,
            "{}-byte write to 0x{:08x} failed\n"
            "ACTUAL:              EXPECTED:\n"
            "pi_error_occurred={}  pi_error_occurred={}\n",
            size, reinterpret_cast<uintptr_t>(ptr + i),
            pi_error_actual, pi_error_expected);
  }
}

static void PIErrorHandler([[maybe_unused]] u32 nIrq, [[maybe_unused]] void* pCtx)
{
  // Clear the interrupt
  s_pi_reg[0] = 0x01;

  s_pi_error_occurred = true;
}

static void Initialize()
{
  network_init();

  // Initialise the video system
  VIDEO_Init();

  // This function initialises the attached controllers
  WPAD_Init();

  // Obtain the preferred video mode from the system
  // This will correspond to the settings in the Wii menu
  GXRModeObj* rmode = VIDEO_GetPreferredMode(NULL);

  // Allocate memory for the display in the uncached region
  void* xfb = MEM_K0_TO_K1(SYS_AllocateFramebuffer(rmode));

  // Initialise the console, required for printf
  console_init(xfb, 20, 20, rmode->fbWidth, rmode->xfbHeight, rmode->fbWidth * VI_DISPLAY_PIX_SZ);

  // Set up the video registers with the chosen mode
  VIDEO_Configure(rmode);

  // Tell the video hardware where our display memory is
  VIDEO_SetNextFramebuffer(xfb);

  // Make the display visible
  VIDEO_SetBlack(FALSE);

  // Flush the video register changes to the hardware
  VIDEO_Flush();

  // Wait for Video setup to complete
  VIDEO_WaitVSync();
  if (rmode->viTVMode & VI_NON_INTERLACE)
    VIDEO_WaitVSync();
}

// Get a pointer to a 64-byte buffer with a page boundary in the middle
u8* GetMEM1Buffer()
{
  const uintptr_t mem1_lo = reinterpret_cast<uintptr_t>(SYS_GetArena1Lo());
  const uintptr_t page_boundary = Common::AlignUp(mem1_lo + CACHE_LINE_SIZE, PAGE_SIZE);
  SYS_SetArena1Lo(reinterpret_cast<void*>(page_boundary + CACHE_LINE_SIZE));
  return reinterpret_cast<u8*>(page_boundary - CACHE_LINE_SIZE);
}

int main()
{
  Initialize();

  network_printf("Setting up PI ERROR interrupt handler...\n");
  IRQ_Request(IRQ_PI_ERROR, PIErrorHandler, nullptr);
  __UnmaskIrq(IM_PI_ERROR);

  network_printf("Allocating memory...\n");
  u8* memory_allocation = GetMEM1Buffer();

  volatile u8* cached_ptr = reinterpret_cast<volatile u8*>(memory_allocation + 16);

  UnalignedStoresTest(cached_ptr, 1, true, Mode::Regular);
  UnalignedStoresTest(cached_ptr, 2, true, Mode::Regular);
  UnalignedStoresTest(cached_ptr, 4, true, Mode::Regular);
  UnalignedStoresTest(cached_ptr, 2, true, Mode::Swap);
  UnalignedStoresTest(cached_ptr, 4, true, Mode::Swap);

  for (size_t size = 1; size <= 32; ++size)
    UnalignedStoresTest(cached_ptr, size, true, Mode::Stswi);

  for (size_t size = 0; size <= 32; ++size)
    UnalignedStoresTest(cached_ptr, size, true, Mode::Stswx);

  network_printf("Invalidating cache...\n");
  asm volatile("dcbi %0, %1" :: "r"(memory_allocation), "r"(0));
  asm volatile("dcbi %0, %1" :: "r"(memory_allocation), "r"(32));

  volatile u8* uncached_ptr = reinterpret_cast<volatile u8*>(MEM_K0_TO_K1(cached_ptr));

  UnalignedStoresTest(uncached_ptr, 1, false, Mode::Regular);
  UnalignedStoresTest(uncached_ptr, 2, false, Mode::Regular);
  UnalignedStoresTest(uncached_ptr, 4, false, Mode::Regular);
  UnalignedStoresTest(uncached_ptr, 2, false, Mode::Swap);
  UnalignedStoresTest(uncached_ptr, 4, false, Mode::Swap);

  for (size_t size = 1; size <= 32; ++size)
    UnalignedStoresTest(cached_ptr, size, false, Mode::Stswi);

  for (size_t size = 0; size <= 32; ++size)
    UnalignedStoresTest(cached_ptr, size, false, Mode::Stswx);

  network_printf("Shutting down...\n");
  network_shutdown();

  return 0;
}
