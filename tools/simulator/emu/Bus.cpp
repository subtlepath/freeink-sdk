// FreeInk emulator — the address space.

#include "Bus.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace freeink::sim::emu {
namespace {

constexpr uint32_t kL1Entries = 4096;  // one per megabyte of a 32-bit space
constexpr uint32_t kPagesPerL1 = 256;  // 1 MB of 4 KB pages

std::string hex(uint32_t value) {
  char buf[16];
  std::snprintf(buf, sizeof(buf), "0x%08X", value);
  return buf;
}

}  // namespace

Bus::Bus(const SocDesc& soc, FlashImage& flash) : soc_(soc), flash_(flash) {
  level1_.resize(kL1Entries);
  mmuTable_.assign(soc_.mmu.entryCount, soc_.mmu.invalidBit);

  // Internal SRAM, with both bus views onto the same bytes.
  for (const SramBlock& block : soc_.sram) {
    if (block.size == 0) break;
    const uint32_t primary = block.iramBase ? block.iramBase : block.dramBase;
    addRam(primary, block.size, block.iramBase ? "IRAM" : "DRAM");
    if (block.iramBase && block.dramBase) aliasRam(primary, block.dramBase, block.size);
  }
  if (soc_.rtcFast.size) addRam(soc_.rtcFast.base, soc_.rtcFast.size, "RTC fast");
  if (soc_.rtcSlow.size) addRam(soc_.rtcSlow.base, soc_.rtcSlow.size, "RTC slow");
  // ROM data. The emulator never executes mask ROM, but apps do read constant
  // tables out of it; backing it with zeroed memory keeps such a read from
  // faulting, at the cost of returning zeros rather than the real table.
  if (soc_.romData.size) addRam(soc_.romData.base, soc_.romData.size, "ROM data");

  // The cached windows are translated through the MMU on every access.
  Page flashPage;
  flashPage.kind = Kind::FlashCache;
  mapPages(soc_.iromCache.base, soc_.iromCache.size, flashPage, false);
  mapPages(soc_.dromCache.base, soc_.dromCache.size, flashPage, false);

  // Mask ROM: mapped so a fetch lands here and is recognised as a ROM call
  // rather than reported as running off into unmapped space.
  Page romPage;
  romPage.kind = Kind::Rom;
  mapPages(soc_.rom.base, soc_.rom.size, romPage, false);
}

Bus::~Bus() = default;

Bus::Page* Bus::pageFor(uint32_t addr) {
  const std::unique_ptr<Page[]>& block = level1_[addr >> 20];
  if (!block) return nullptr;
  Page* page = &block[(addr >> kPageBits) & (kPagesPerL1 - 1)];
  return page->kind == Kind::Unmapped ? nullptr : page;
}

const Bus::Page* Bus::pageFor(uint32_t addr) const {
  const std::unique_ptr<Page[]>& block = level1_[addr >> 20];
  if (!block) return nullptr;
  const Page* page = &block[(addr >> kPageBits) & (kPagesPerL1 - 1)];
  return page->kind == Kind::Unmapped ? nullptr : page;
}

void Bus::mapPages(uint32_t base, uint32_t size, const Page& prototype, bool ramWalk) {
  for (uint64_t addr = base; addr < static_cast<uint64_t>(base) + size; addr += kPageSize) {
    const uint32_t at = static_cast<uint32_t>(addr);
    std::unique_ptr<Page[]>& block = level1_[at >> 20];
    if (!block) block.reset(new Page[kPagesPerL1]);
    Page& page = block[(at >> kPageBits) & (kPagesPerL1 - 1)];
    page = prototype;
    if (ramWalk && prototype.host) page.host = prototype.host + (addr - base);
  }
}

void Bus::addRam(uint32_t base, uint32_t size, const char* name) {
  auto region = std::make_unique<RamRegion>();
  region->base = base;
  region->size = size;
  region->name = name;
  // Padded so a load that straddles the very end of a region reads padding
  // instead of walking off the host allocation; the access is still a firmware
  // bug, and `describe()` still names the region, but it is not a crash.
  region->storage.assign(static_cast<size_t>(size) + 8, 0);
  region->backing = &region->storage;

  Page prototype;
  prototype.kind = Kind::Ram;
  prototype.host = region->backing->data();
  mapPages(base, size, prototype, true);
  ram_.push_back(std::move(region));
}

std::vector<Bus::RamRange> Bus::ramRegions() const {
  std::vector<RamRange> out;
  for (const auto& region : ram_) {
    out.push_back({region->base, region->size, region->storage.empty()});
  }
  return out;
}

void Bus::aliasRam(uint32_t existingBase, uint32_t aliasBase, uint32_t size) {
  RamRegion* source = nullptr;
  for (auto& region : ram_) {
    if (region->base == existingBase) {
      source = region.get();
      break;
    }
  }
  if (!source) return;

  auto alias = std::make_unique<RamRegion>();
  alias->base = aliasBase;
  alias->size = size;
  alias->name = source->name + " (alias)";
  alias->backing = source->backing;

  Page prototype;
  prototype.kind = Kind::Ram;
  prototype.host = alias->backing->data();
  mapPages(aliasBase, size, prototype, true);
  ram_.push_back(std::move(alias));
}

void Bus::attach(uint32_t base, uint32_t size, std::unique_ptr<MmioDevice> device) {
  Page prototype;
  prototype.kind = Kind::Mmio;
  prototype.device = device.get();
  prototype.deviceBase = base;
  mapPages(base, size, prototype, false);
  devices_.push_back(std::move(device));
}

void Bus::raise(FaultKind kind, uint32_t address, const std::string& detail) {
  if (fault_.kind != FaultKind::None) return;  // keep the first fault, not the last
  fault_.kind = kind;
  fault_.address = address;
  fault_.pc = pc_;
  fault_.detail = detail;
}

uint32_t Bus::mmuIndexFor(uint32_t vaddr) const {
  return (vaddr & soc_.mmu.linearMask) / soc_.mmu.pageSize;
}

bool Bus::translate(uint32_t vaddr, uint32_t* paddr) const {
  const uint32_t index = mmuIndexFor(vaddr);
  if (index >= mmuTable_.size()) return false;
  const uint32_t entry = mmuTable_[index];
  if (entry & soc_.mmu.invalidBit) return false;
  if (soc_.mmu.typeBit && (entry & soc_.mmu.typeBit)) return false;  // PSRAM, not flash
  const uint32_t page = entry & soc_.mmu.valueMask;
  *paddr = page * soc_.mmu.pageSize + (vaddr & (soc_.mmu.pageSize - 1));
  return true;
}

void Bus::setMmuEntry(uint32_t index, uint32_t value) {
  if (index < mmuTable_.size()) mmuTable_[index] = value;
}

uint32_t Bus::mmuEntry(uint32_t index) const {
  return index < mmuTable_.size() ? mmuTable_[index] : soc_.mmu.invalidBit;
}

bool Bus::mapFlash(uint32_t vaddr, uint32_t paddr, uint32_t length, std::string* error) {
  const uint32_t pageSize = soc_.mmu.pageSize;
  if ((vaddr & (pageSize - 1)) != (paddr & (pageSize - 1))) {
    if (error) {
      *error = "cannot map " + hex(paddr) + " at " + hex(vaddr) +
               ": a flash page can only be mapped at the same offset within a page";
    }
    return false;
  }
  const uint32_t firstV = vaddr & ~(pageSize - 1);
  const uint32_t firstP = paddr & ~(pageSize - 1);
  const uint32_t pages = (length + (vaddr - firstV) + pageSize - 1) / pageSize;
  for (uint32_t i = 0; i < pages; ++i) {
    const uint32_t index = mmuIndexFor(firstV + i * pageSize);
    if (index >= mmuTable_.size()) {
      if (error) *error = "flash mapping at " + hex(firstV + i * pageSize) + " is outside the MMU window";
      return false;
    }
    mmuTable_[index] = (firstP / pageSize) + i;
  }
  return true;
}

// ── Access ───────────────────────────────────────────────────────────────────

namespace {
uint32_t extract(uint32_t word, uint32_t byteOffset, uint32_t bytes) {
  const uint32_t shift = byteOffset * 8;
  const uint32_t mask = bytes >= 4 ? 0xFFFFFFFFu : ((1u << (bytes * 8)) - 1);
  return (word >> shift) & mask;
}
}  // namespace

uint32_t Bus::read32(uint32_t addr) {
  const Page* page = pageFor(addr);
  if (!page) {
    ++unmappedReads_;
    raise(FaultKind::Unmapped, addr, "load from unmapped " + describe(addr));
    return 0;
  }
  const uint32_t offset = addr & (kPageSize - 1);
  switch (page->kind) {
    case Kind::Ram: {
      if (offset + 4 <= kPageSize) {
        uint32_t value;
        std::memcpy(&value, page->host + offset, 4);
        return value;
      }
      break;  // straddles a page: fall through to the byte path
    }
    case Kind::Mmio: {
      const uint32_t regOffset = addr - page->deviceBase;
      const uint32_t word = page->device->read(regOffset & ~3u);
      return (regOffset & 3) == 0 ? word : extract(word, regOffset & 3, 4);
    }
    case Kind::FlashCache: {
      uint32_t paddr = 0;
      if (!translate(addr, &paddr)) {
        raise(FaultKind::MmuInvalid, addr, "load from unmapped flash page at " + describe(addr));
        return 0;
      }
      uint8_t bytes[4];
      flash_.read(paddr, bytes, 4);
      uint32_t value;
      std::memcpy(&value, bytes, 4);
      return value;
    }
    case Kind::Rom:
      // Reading ROM *code* (rather than calling it) usually means a firmware
      // that inspects a ROM function's bytes. Nothing sensible to return.
      raise(FaultKind::Unimplemented, addr,
            "read of mask ROM code at " + hex(addr) + " — the emulator intercepts ROM calls "
            "rather than storing ROM bytes");
      return 0;
    default:
      break;
  }

  uint32_t value = 0;
  for (int i = 0; i < 4; ++i) value |= static_cast<uint32_t>(read8(addr + i)) << (8 * i);
  return value;
}

uint16_t Bus::read16(uint32_t addr) {
  const Page* page = pageFor(addr);
  const uint32_t offset = addr & (kPageSize - 1);
  if (page && page->kind == Kind::Ram && offset + 2 <= kPageSize) {
    uint16_t value;
    std::memcpy(&value, page->host + offset, 2);
    return value;
  }
  // A halfword that straddles two words has to be assembled from both. Only
  // an aligned read is composed from one: shifting a single word right by 24
  // and calling it 16 bits loses the high byte, which on a byte-aligned
  // instruction set is most of an instruction.
  const uint32_t low = read32(addr & ~3u) >> ((addr & 3) * 8);
  if ((addr & 3) != 3) return static_cast<uint16_t>(low);
  return static_cast<uint16_t>(low | (read32((addr & ~3u) + 4) << 8));
}

uint8_t Bus::read8(uint32_t addr) {
  const Page* page = pageFor(addr);
  if (page && page->kind == Kind::Ram) return page->host[addr & (kPageSize - 1)];
  return static_cast<uint8_t>(read32(addr & ~3u) >> ((addr & 3) * 8));
}

void Bus::write32(uint32_t addr, uint32_t value) {
  Page* page = pageFor(addr);
  if (!page) {
    raise(FaultKind::Unmapped, addr, "store to unmapped " + describe(addr));
    return;
  }
  const uint32_t offset = addr & (kPageSize - 1);
  switch (page->kind) {
    case Kind::Ram:
      if (offset + 4 <= kPageSize) {
        std::memcpy(page->host + offset, &value, 4);
        return;
      }
      break;
    case Kind::Mmio:
      page->device->write((addr - page->deviceBase) & ~3u, value, 0xFFFFFFFFu);
      return;
    case Kind::FlashCache:
      // The cached windows are read-only on hardware: a store there is a
      // firmware bug (a const pointer written through), and silently dropping
      // it would hide exactly the bug worth finding.
      raise(FaultKind::Unmapped, addr, "store to read-only flash-mapped memory at " + describe(addr));
      return;
    case Kind::Rom:
      raise(FaultKind::Unmapped, addr, "store to mask ROM at " + hex(addr));
      return;
    default:
      break;
  }
  for (int i = 0; i < 4; ++i) write8(addr + i, static_cast<uint8_t>(value >> (8 * i)));
}

void Bus::write16(uint32_t addr, uint16_t value) {
  Page* page = pageFor(addr);
  const uint32_t offset = addr & (kPageSize - 1);
  if (page && page->kind == Kind::Ram && offset + 2 <= kPageSize) {
    std::memcpy(page->host + offset, &value, 2);
    return;
  }
  if (page && page->kind == Kind::Mmio) {
    const uint32_t regOffset = addr - page->deviceBase;
    const uint32_t shift = (regOffset & 3) * 8;
    page->device->write(regOffset & ~3u, static_cast<uint32_t>(value) << shift, 0xFFFFu << shift);
    return;
  }
  write8(addr, static_cast<uint8_t>(value));
  write8(addr + 1, static_cast<uint8_t>(value >> 8));
}

void Bus::write8(uint32_t addr, uint8_t value) {
  Page* page = pageFor(addr);
  if (page && page->kind == Kind::Ram) {
    page->host[addr & (kPageSize - 1)] = value;
    return;
  }
  if (page && page->kind == Kind::Mmio) {
    const uint32_t regOffset = addr - page->deviceBase;
    const uint32_t shift = (regOffset & 3) * 8;
    page->device->write(regOffset & ~3u, static_cast<uint32_t>(value) << shift, 0xFFu << shift);
    return;
  }
  if (!page) {
    raise(FaultKind::Unmapped, addr, "store to unmapped " + describe(addr));
    return;
  }
  write32(addr & ~3u, (read32(addr & ~3u) & ~(0xFFu << ((addr & 3) * 8))) |
                          (static_cast<uint32_t>(value) << ((addr & 3) * 8)));
}

const uint8_t* Bus::hostRead(uint32_t addr, uint32_t length) {
  const Page* page = pageFor(addr);
  if (!page) return nullptr;
  const uint32_t offset = addr & (kPageSize - 1);
  if (page->kind == Kind::Ram) {
    // Contiguity beyond the page is guaranteed by the region walk in addRam:
    // consecutive pages of a region point into one allocation.
    return page->host + offset;
  }
  if (page->kind == Kind::FlashCache) {
    uint32_t paddr = 0;
    if (!translate(addr, &paddr)) return nullptr;
    // Only within one 64 KB flash page is the mapping contiguous.
    const uint32_t pageRemaining = soc_.mmu.pageSize - (addr & (soc_.mmu.pageSize - 1));
    if (length > pageRemaining) return nullptr;
    if (paddr + length > flash_.size()) return nullptr;
    return flash_.bytes() + paddr;
  }
  return nullptr;
}

uint8_t* Bus::hostWrite(uint32_t addr, uint32_t length) {
  Page* page = pageFor(addr);
  if (!page || page->kind != Kind::Ram) return nullptr;
  (void)length;
  return page->host + (addr & (kPageSize - 1));
}

bool Bus::peek(uint32_t addr, uint32_t length, std::vector<uint8_t>* out) {
  const Fault saved = fault_;
  out->clear();
  out->reserve(length);
  for (uint32_t i = 0; i < length; ++i) {
    const Page* page = pageFor(addr + i);
    if (!page) {
      fault_ = saved;
      return false;
    }
    out->push_back(read8(addr + i));
  }
  fault_ = saved;  // a debugger read must not arm a fault the firmware never took
  return true;
}

std::string Bus::describe(uint32_t addr) const {
  const Page* page = pageFor(addr);
  if (page) {
    switch (page->kind) {
      case Kind::Ram:
        for (const auto& region : ram_) {
          if (addr >= region->base && addr - region->base < region->size) {
            char buf[96];
            std::snprintf(buf, sizeof(buf), "%s+0x%X (%s)", region->name.c_str(), addr - region->base,
                          hex(addr).c_str());
            return buf;
          }
        }
        return "RAM " + hex(addr);
      case Kind::Mmio: {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s+0x%X (%s)", page->device->name(), addr - page->deviceBase,
                      hex(addr).c_str());
        return buf;
      }
      case Kind::FlashCache: {
        const uint32_t index = mmuIndexFor(addr);
        uint32_t paddr = 0;
        const bool mapped = translate(addr, &paddr);
        char buf[160];
        const char* window = inRange(soc_.iromCache, addr) ? "IROM" : "DROM";
        if (mapped) {
          std::snprintf(buf, sizeof(buf), "%s %s (MMU page %u -> flash 0x%06X)", window, hex(addr).c_str(),
                        index, paddr);
        } else {
          std::snprintf(buf, sizeof(buf), "%s %s (MMU page %u is not mapped)", window, hex(addr).c_str(), index);
        }
        return buf;
      }
      case Kind::Rom:
        return "mask ROM " + hex(addr);
      default:
        break;
    }
  }
  return "address " + hex(addr) + " (no memory or peripheral is mapped there)";
}

}  // namespace freeink::sim::emu
