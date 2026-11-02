/*
 * CC1101 HAL - RF433-1 / Nartis D101, 433.3-434.7 MHz.
 *
 * REMINDER, because this file looks almost identical to the one in the sibling
 * RF433-2 projects and is not: RF433-1 is MSB-FIRST on air. There is no call to
 * RadioHal::reverse8() anywhere below and there must not be one.
 */

#include "cc1101_hal.h"

#include "esphome/core/log.h"
#include "esphome/core/hal.h"  // esphome::delay / delayMicroseconds / micros / millis / yield

#include <cinttypes>

namespace esphome::nartis_rf_meter {

static const char *const TAG = "cc1101_hal";

// ---------------- bit-banged 4-wire SPI (mode 0, MSB-first) ----------------
//
// The CC1101 clocks MOSI in on the rising edge of SCLK and drives MISO on the
// falling edge, i.e. CPOL = 0, CPHA = 0, so one bit is: SCLK low, present the
// output bit, SCLK high, sample the input bit. ESPHome's `spi` component is
// deliberately not used - the CMT2300A backend is self-contained for the same
// reason, and keeping both that way means a YAML that selects one radio pulls in
// nothing belonging to the other, and no shared-bus arbitration can land in the
// middle of a half-duplex request/reply turn.

uint8_t Cc1101Hal::xfer_byte_(uint8_t out) {
  uint8_t in = 0;
  for (int8_t i = 7; i >= 0; i--) {
    pin_low(this->sclk_);
    this->mosi_.digital_write(((out >> i) & 1) != 0);
    esphome::delayMicroseconds(SPI_HALF_PERIOD_US);
    pin_high(this->sclk_);
    in = (uint8_t) (in << 1);
    if (pin_read(this->miso_)) {
      in |= 1;
    }
    esphome::delayMicroseconds(SPI_HALF_PERIOD_US);
  }
  pin_low(this->sclk_);
  return in;
}

// CHIP_RDYn. The chip holds MISO high while the crystal is still starting up or a
// reset is in progress; the datasheet requires waiting for it to fall before the
// first bit of any access, not just the first access after power-up.
bool Cc1101Hal::wait_miso_low_(uint32_t timeout_us) {
  const uint32_t t0 = esphome::micros();
  while (pin_read(this->miso_)) {
    if (esphome::micros() - t0 > timeout_us) {
      return false;
    }
  }
  return true;
}

// Every access starts here, so this is the one place that can observe CHIP_RDYn
// failing - and TI requires SO to be low before the header byte is clocked, because
// that is the chip reporting its SPI interface and crystal are up. Clocking anyway
// can return arbitrary register values, execute a strobe that was never intended, or
// report a state transition that never happened.
//
// The failure is latched in spi_not_ready_ rather than returned. A bool here would be
// ignored at every one of the call sites below - threading a status out of
// write_reg_() would bury write_config_()'s forty-odd register writes in error
// handling for a fault that is all-or-nothing in practice - and an ignored return is
// exactly how this failure goes missing. With no return value there is nothing to
// forget to check; init(), drain_rx() and read_rx_exact() clear the flag before a
// sequence and test it afterwards.
void Cc1101Hal::select_() {
  pin_low(this->cs_);
  if (!this->wait_miso_low_(CHIP_RDY_TIMEOUT_US)) {
    this->spi_not_ready_ = true;
  }
}

void Cc1101Hal::deselect_() {
  pin_high(this->cs_);
  esphome::delayMicroseconds(SPI_HALF_PERIOD_US);
}

// A strobe is a bare header byte; what comes back on MISO during it is the chip
// status byte (CHIP_RDYn, state, FIFO bytes available), which is the cheapest
// state read there is.
uint8_t Cc1101Hal::strobe_(uint8_t cmd) {
  this->select_();
  const uint8_t status = this->xfer_byte_(cmd);
  this->deselect_();
  return status;
}

void Cc1101Hal::write_reg_(uint8_t addr, uint8_t value) {
  this->select_();
  this->xfer_byte_((uint8_t) (addr | CC1101_HDR_WRITE));
  this->xfer_byte_(value);
  this->deselect_();
}

void Cc1101Hal::write_burst_(uint8_t addr, const uint8_t *data, size_t len) {
  if (data == nullptr || len == 0) {
    return;
  }
  this->select_();
  this->xfer_byte_((uint8_t) (addr | CC1101_HDR_WRITE | CC1101_HDR_BURST));
  for (size_t i = 0; i < len; i++) {
    this->xfer_byte_(data[i]);
  }
  this->deselect_();
}

uint8_t Cc1101Hal::read_reg_(uint8_t addr) {
  this->select_();
  this->xfer_byte_((uint8_t) (addr | CC1101_HDR_READ));
  const uint8_t value = this->xfer_byte_(0x00);
  this->deselect_();
  return value;
}

// Status registers live at 0x30-0x3D, which is exactly the command-strobe range.
// Without the burst bit the chip would execute a strobe instead of returning the
// register, so a read of anything in that range MUST set it.
uint8_t Cc1101Hal::read_burst_reg_once_(uint8_t addr) {
  this->select_();
  this->xfer_byte_((uint8_t) (addr | CC1101_HDR_READ | CC1101_HDR_BURST));
  const uint8_t value = this->xfer_byte_(0x00);
  this->deselect_();
  return value;
}

// CC1101 errata, "Register read with SPI": a status register that updates while it
// is being shifted out can return a byte that was never a valid value - the classic
// symptom is a bogus RXBYTES count that makes the driver read past the end of the
// FIFO. The documented workaround is to read until two consecutive reads agree.
uint8_t Cc1101Hal::read_status_reg_(uint8_t addr) {
  uint8_t previous = this->read_burst_reg_once_(addr);
  for (uint8_t i = 0; i < STATUS_READ_RETRIES; i++) {
    const uint8_t current = this->read_burst_reg_once_(addr);
    if (current == previous) {
      return current;
    }
    previous = current;
  }
  // Six disagreements in a row means something is genuinely churning (a FIFO filling
  // fast, or a wiring problem); the last value is still the freshest one we have.
  ESP_LOGV(TAG, "status register 0x%02X never settled, using 0x%02X", addr, previous);
  return previous;
}

void Cc1101Hal::read_fifo_burst_(uint8_t *data, size_t len) {
  if (data == nullptr || len == 0) {
    return;
  }
  this->select_();
  this->xfer_byte_((uint8_t) (CC1101_REG_FIFO | CC1101_HDR_READ | CC1101_HDR_BURST));
  for (size_t i = 0; i < len; i++) {
    data[i] = this->xfer_byte_(0x00);
  }
  this->deselect_();
}

// ---------------- state machine ----------------

uint8_t Cc1101Hal::marcstate_() { return this->read_status_reg_(CC1101_REG_MARCSTATE) & CC1101_MARCSTATE_MASK; }

bool Cc1101Hal::wait_marcstate_(uint8_t target, uint32_t timeout_ms) {
  const uint32_t start = esphome::millis();
  while (esphome::millis() - start < timeout_ms) {
    if (this->marcstate_() == target) {
      return true;
    }
    esphome::delayMicroseconds(STATE_POLL_INTERVAL_US);
  }
  return false;
}

bool Cc1101Hal::go_idle_() {
  // Leaving RX ends the epoch whether the caller thought of it that way or not, so
  // the flag is cleared here rather than in each of the handful of callers - this is
  // the one funnel every exit from RX passes through (go_standby(), the start of
  // transmit(), and begin_rx()'s own teardown).
  this->rx_epoch_armed_ = false;
  this->strobe_(CC1101_STROBE_SIDLE);
  return this->wait_marcstate_(CC1101_MARC_IDLE, STATE_TIMEOUT_MS);
}

// IDLE is the lowest state that keeps the whole register profile alive. SPWD is
// lower still, and this component idles for minutes between poll cycles so the
// saving would be real - but it stops the crystal, needs a CS-toggle wake-up
// handshake, loses PATABLE entries 1-7 and costs a full recalibration on the way
// out, none of which has been exercised on hardware for this backend. See the note
// in cc1101_hal.h about why go_sleep() is left at the RadioHal default.
bool Cc1101Hal::go_standby() { return this->go_idle_(); }

// ---------------- pins ----------------

void Cc1101Hal::set_pins(esphome::InternalGPIOPin *sclk, esphome::InternalGPIOPin *miso,
                         esphome::InternalGPIOPin *mosi, esphome::InternalGPIOPin *cs,
                         esphome::InternalGPIOPin *gdo0) {
  this->sclk_pin_ = sclk;
  this->miso_pin_ = miso;
  this->mosi_pin_ = mosi;
  this->cs_pin_ = cs;
  this->gdo0_pin_ = gdo0;
  // Guarded individually: init() reports a missing required pin with a message the
  // user can act on, and must be reachable rather than crashed past.
  if (sclk != nullptr) {
    this->sclk_ = sclk->to_isr();
  }
  if (miso != nullptr) {
    this->miso_ = miso->to_isr();
  }
  if (mosi != nullptr) {
    this->mosi_ = mosi->to_isr();
  }
  if (cs != nullptr) {
    this->cs_ = cs->to_isr();
  }
  // gdo0 is optional: every path that touches gdo0_ is guarded on gdo0_pin_.
  if (gdo0 != nullptr) {
    this->gdo0_ = gdo0->to_isr();
  }
}

// ---------------- parameter solving ----------------

void Cc1101Hal::set_frequency(uint32_t freq_hz) { this->freq_hz_ = freq_hz; }

void Cc1101Hal::set_rx_frequency(uint32_t freq_hz) { this->rx_freq_hz_ = freq_hz; }

// The RX carrier actually tuned. The offset is validated to +-200 kHz in YAML, so
// the clamp below cannot fire in practice; it is here so that no arrangement of
// setter calls can hand program_frequency_() a wrapped-around unsigned value.
uint32_t Cc1101Hal::rx_tuned_hz_() const {
  const int64_t hz = (int64_t) this->rx_freq_hz_ + (int64_t) this->rx_offset_hz_;
  if (hz <= 0) {
    return 1u;
  }
  return (uint32_t) hz;
}

uint8_t Cc1101Hal::mdmcfg2_value_() const {
  const uint8_t mod =
      (this->modulation_ == Modulation::GFSK) ? CC1101_MDMCFG2_MOD_GFSK : CC1101_MDMCFG2_MOD_2FSK;
  // DEM_DCFILT_OFF = 0 and MANCHESTER_EN = 0 are both encoded as absent bits here.
  // SYNC_MODE is always 16/16 exact. Sixteen bits is all this PHY needs - the on-air
  // sync is 55 55 F6 72 and its leading 55 55 merges with the preamble - and it is
  // also all the part offers, since SYNC_MODE = 011/111 repeats SYNC1:SYNC0 rather
  // than taking a second register. See the sync-word note in cc1101_defs.h.
  return (uint8_t) (mod | CC1101_MDMCFG2_SYNC_16_16);
}

// FREQ = f_carrier * 2^16 / Fxosc, one step being Fxosc / 2^16 = 396.728 Hz. This is
// computed rather than tabled: TX and RX are independent carriers here, the channel
// can change between exchanges, and the RX offset is a free parameter, so there is
// no fixed set of words to table.
//
// It is the CARRIER that goes in, with no IF term of any kind. The CMT2300A is a
// low-IF receiver and its register blobs hold an LO that sits CHANNEL_LOW_IF_HZ
// above the carrier (see radio_hal.h); the CC1101 is not, and applying that offset
// here would put the receiver 282 kHz off the meter.
uint32_t Cc1101Hal::freq_word_for_(uint32_t hz) {
  return (uint32_t) ((((uint64_t) hz << 16) + (CC1101_XTAL_HZ / 2)) / CC1101_XTAL_HZ);
}

uint32_t Cc1101Hal::freq_hz_for_word_(uint32_t word) {
  return (uint32_t) ((((uint64_t) word * CC1101_XTAL_HZ) + (1u << 15)) >> 16);
}

void Cc1101Hal::program_frequency_(uint32_t hz) {
  const uint32_t word = freq_word_for_(hz);
  // FREQ2<7:6> are always zero; the synthesiser word is 22 bits wide.
  this->write_reg_(CC1101_REG_FREQ2, (uint8_t) ((word >> 16) & 0x3F));
  this->write_reg_(CC1101_REG_FREQ1, (uint8_t) ((word >> 8) & 0xFF));
  this->write_reg_(CC1101_REG_FREQ0, (uint8_t) (word & 0xFF));
}

void Cc1101Hal::solve_params_() {
  // Deviation: f_dev = (Fxosc / 2^17) * (8 + DEV_M) * 2^DEV_E, with DEVIATION_E and
  // DEVIATION_M both 3 bits. Searching all 64 combinations costs nothing at boot and
  // leaves no room for an arithmetic slip. At the 6000 Hz default it lands on
  // DEV_E = 1, DEV_M = 7 -> 5950.9 Hz (DEVIATN = 0x17); the neighbours are
  // DEV_E = 1, DEV_M = 6 -> 5553.5 Hz and DEV_E = 2, DEV_M = 0 -> 6347.6 Hz, both
  // further from the SDR-measured +-6.0 kHz.
  //
  // This search is also the reason the driver does not carry the "DEVIATN ~= 0x35"
  // hint from fw/RF_PHY_SPEC.md sec. 4.2/sec. 5: that byte is DEV_E = 3, DEV_M = 5 = 20629 Hz,
  // three and a half times the measured deviation, and the (E = 1, M = 5) decomposition
  // the spec prints beside it is 5157 Hz, not the 5.93 kHz it claims. The solver gets
  // the right answer from the measured Hz without being told any register value at
  // all, which is precisely why it is a solver. Do not replace it with 0x35.
  uint32_t best_err = UINT32_MAX;
  for (uint8_t e = 0; e < 8; e++) {
    for (uint8_t m = 0; m < 8; m++) {
      const uint32_t f = (uint32_t) ((((uint64_t) CC1101_XTAL_HZ * (8u + m)) << e) >> 17);
      const uint32_t err = (f > this->deviation_hz_) ? (f - this->deviation_hz_) : (this->deviation_hz_ - f);
      if (err < best_err) {
        best_err = err;
        this->dev_e_ = e;
        this->dev_m_ = m;
        this->deviation_actual_hz_ = f;
      }
    }
  }

  // RX channel filter: BW = Fxosc / (8 * (4 + CHANBW_M) * 2^CHANBW_E), both fields
  // 2 bits, so 16 settings spanning 58035 Hz (E=3, M=3 - the chip's floor) to
  // 812500 Hz. The 101562 Hz default lands on E = 3, M = 0, and is chosen for the
  // crystal-offset budget rather than for the signal: Carson bandwidth here is only
  // ~17 kHz, but frequency offset compensation reaches just +-BW/4 and a worst-case
  // 20 ppm + 20 ppm pair at 433.8 MHz is +-17.4 kHz. See
  // CC1101_DEFAULT_RX_BANDWIDTH_HZ for the full reasoning.
  best_err = UINT32_MAX;
  for (uint8_t e = 0; e < 4; e++) {
    for (uint8_t m = 0; m < 4; m++) {
      const uint32_t bw = CC1101_XTAL_HZ / ((8u * (4u + m)) << e);
      const uint32_t err =
          (bw > this->rx_bandwidth_hz_) ? (bw - this->rx_bandwidth_hz_) : (this->rx_bandwidth_hz_ - bw);
      if (err < best_err) {
        best_err = err;
        this->chanbw_e_ = e;
        this->chanbw_m_ = m;
        this->rx_bandwidth_actual_hz_ = bw;
      }
    }
  }
  // TEST2/TEST1 are keyed on the RX filter bandwidth, and TI characterises exactly
  // two pairs: 0x88/0x31 strictly above 101 kHz and 0x81/0x35 at or below it. The
  // bandwidth is a YAML tunable spanning the chip's full 58...812 kHz range, so a
  // fixed pair would leave most of that range running registers specified for a
  // different filter. The default 101562 Hz sits 562 Hz above the threshold and so
  // picks the WIDE pair - that is correct and intended, not a rounding accident.
  if (this->rx_bandwidth_actual_hz_ > CC1101_TEST_REG_BW_THRESHOLD_HZ) {
    this->test2_ = CC1101_TEST2_WIDE_BW;
    this->test1_ = CC1101_TEST1_WIDE_BW;
  } else {
    this->test2_ = CC1101_TEST2_NARROW_BW;
    this->test1_ = CC1101_TEST1_NARROW_BW;
  }

  // FSCTRL1.FREQ_IF is fixed at 152.34 kHz in CC1101_CONFIG, which is what SmartRF
  // picks for every bandwidth up to ~100 kHz and leaves plenty of headroom over the
  // 50.8 kHz the default filter needs. The bandwidth knob reaches 812 kHz, though,
  // and past ~305 kHz the IF no longer clears half the filter - the image folds into
  // the passband and sensitivity collapses. That is a configuration no one on this
  // PHY should want (the signal is 17 kHz wide), so it is a warning rather than a
  // solved-for IF: say it loudly and let the user explain themselves.
  if (this->rx_bandwidth_actual_hz_ / 2u > CC1101_IF_HZ) {
    ESP_LOGW(TAG,
             "RX bandwidth %" PRIu32 " Hz is wider than twice the fixed %" PRIu32
             " Hz IF - the image band folds into the passband and sensitivity will suffer. This PHY "
             "occupies only ~17 kHz; use the default 101562 Hz unless you have a measured reason not to.",
             this->rx_bandwidth_actual_hz_, CC1101_IF_HZ);
  }

  this->pa_value_ = cc1101_pa_setting_for_dbm(this->output_power_dbm_, &this->output_power_actual_dbm_);
  if (this->output_power_actual_dbm_ != this->output_power_dbm_) {
    ESP_LOGW(TAG, "output power %d dBm is not a 433 MHz PATABLE entry - using the nearest, %d dBm",
             this->output_power_dbm_, this->output_power_actual_dbm_);
  }
}

void Cc1101Hal::write_config_() {
  for (size_t i = 0; i < CC1101_CONFIG_SIZE; i++) {
    this->write_reg_(CC1101_CONFIG[i].addr, CC1101_CONFIG[i].value);
  }

  // MDMCFG4 packs the RX filter together with the data-rate exponent, so it can only
  // be written once both are known: CHANBW_E<7:6> | CHANBW_M<5:4> | DRATE_E<3:0>.
  this->write_reg_(CC1101_REG_MDMCFG4,
                   (uint8_t) ((this->chanbw_e_ << 6) | (this->chanbw_m_ << 4) | CC1101_DRATE_E));
  // MDMCFG2 carries the modulation tunable alongside the sync mode.
  this->write_reg_(CC1101_REG_MDMCFG2, this->mdmcfg2_value_());
  // The sync word is a tunable too (`sync_word`), so it is written here rather than
  // from CC1101_CONFIG. High byte into SYNC1 goes out first: the default 0xF672 puts
  // F6 then 72 on the air, which with NUM_PREAMBLE = 12 reproduces the CIU's
  // 55 x12 | F6 72 exactly. See the sync-word note in cc1101_defs.h.
  this->write_reg_(CC1101_REG_SYNC1, (uint8_t) (this->sync_word_ >> 8));
  this->write_reg_(CC1101_REG_SYNC0, (uint8_t) (this->sync_word_ & 0xFFu));
  // DEVIATN: DEVIATION_E<6:4> | DEVIATION_M<2:0>. One value serves both directions -
  // RF433-1 is symmetric, with the same +-6 kHz measured on the CIU's transmission
  // and on the meter's reply.
  this->write_reg_(CC1101_REG_DEVIATN, (uint8_t) ((this->dev_e_ << 4) | this->dev_m_));
  // TEST2/TEST1 track the RX filter bandwidth and FOCCFG carries a tunable bit, so
  // all three are written here rather than from CC1101_CONFIG. They are written
  // AFTER the table loop on purpose - the table no longer holds them, and keeping
  // the order "table first, solved values second" means a future entry slipping back
  // into the table could not silently win over the solved one.
  this->write_reg_(CC1101_REG_TEST2, this->test2_);
  this->write_reg_(CC1101_REG_TEST1, this->test1_);
  this->write_reg_(CC1101_REG_FOCCFG, this->foccfg_value_());
  this->program_frequency_(this->freq_hz_);

  // FREND0.PA_POWER is 0, so the PA only ever reads PATABLE[0]; the remaining
  // entries are zeroed so that a stray index could only mean "off", never
  // "unexpectedly loud". A burst write to 0x3E starts at index 0 and auto-increments.
  uint8_t patable[CC1101_PATABLE_SIZE] = {};
  patable[0] = this->pa_value_;
  this->write_burst_(CC1101_REG_PATABLE, patable, CC1101_PATABLE_SIZE);
}

// ================= INIT (once at startup) =================
bool Cc1101Hal::init() {
  if (this->sclk_pin_ == nullptr || this->miso_pin_ == nullptr || this->mosi_pin_ == nullptr ||
      this->cs_pin_ == nullptr) {
    ESP_LOGE(TAG, "SPI pins not set - set_pins() must be called before init()");
    return false;
  }

  this->sclk_pin_->setup();
  this->miso_pin_->setup();
  this->mosi_pin_->setup();
  this->cs_pin_->setup();
  this->sclk_.pin_mode(gpio::FLAG_OUTPUT);
  this->mosi_.pin_mode(gpio::FLAG_OUTPUT);
  this->cs_.pin_mode(gpio::FLAG_OUTPUT);
  this->miso_.pin_mode(gpio::FLAG_INPUT);
  if (this->gdo0_pin_ != nullptr) {
    this->gdo0_pin_->setup();
    this->gdo0_.pin_mode(gpio::FLAG_INPUT);
  }

  // Manual power-up reset, CC1101 datasheet 19.1.2. The timing matters: the chip
  // needs CSn high for at least 40 us after the short low pulse before it will
  // accept SRES, and MISO must be seen low (CHIP_RDYn) both before the strobe and
  // after it, the second time meaning the reset has finished.
  pin_high(this->cs_);
  pin_high(this->sclk_);
  pin_low(this->mosi_);
  esphome::delay(1);
  pin_low(this->cs_);
  esphome::delayMicroseconds(10);
  pin_high(this->cs_);
  esphome::delayMicroseconds(45);
  pin_low(this->cs_);
  if (!this->wait_miso_low_(CHIP_RDY_TIMEOUT_US)) {
    pin_high(this->cs_);
    ESP_LOGE(TAG, "CC1101 not responding: MISO never went low - check SPI wiring and 3V3 supply");
    return false;
  }
  this->xfer_byte_(CC1101_STROBE_SRES);
  if (!this->wait_miso_low_(RESET_TIMEOUT_US)) {
    pin_high(this->cs_);
    ESP_LOGE(TAG, "CC1101 did not complete SRES - MISO stuck high");
    return false;
  }
  pin_high(this->cs_);
  esphome::delay(1);

  // From here on everything goes through select_(), which raises spi_not_ready_ if
  // CHIP_RDYn never appears. Start from a clean slate so the checks below can only
  // be reporting on this init.
  this->clear_spi_error_();

  // Wiring sanity check. A genuine CC1101 answers PARTNUM 0x00 / VERSION 0x14, but
  // VERSION tracks the die revision (0x04 and 0x17 have both been seen) and clone
  // parts answer with their own numbers, so only an all-zero or all-ones VERSION -
  // which is what a floating or shorted MISO looks like - is treated as fatal.
  const uint8_t partnum = this->read_status_reg_(CC1101_REG_PARTNUM);
  const uint8_t version = this->read_status_reg_(CC1101_REG_VERSION);
  if (this->spi_error_()) {
    // CHIP_RDYn never went low for one of those reads, so whatever came back on MISO
    // was shifted out of a chip that had not said it was ready - it is not evidence
    // of anything, and judging PARTNUM/VERSION on it would turn a wiring or supply
    // fault into a confident "clone part, continuing".
    ESP_LOGE(TAG, "CC1101 never asserted CHIP_RDYn (MISO stayed high for %" PRIu32
                  " us) while probing - check SPI wiring, CS and the 3V3 supply",
             CHIP_RDY_TIMEOUT_US);
    return false;
  }
  ESP_LOGD(TAG, "PARTNUM: 0x%02X (expected 0x%02X), VERSION: 0x%02X (expected 0x%02X)", partnum,
           CC1101_PARTNUM_EXPECTED, version, CC1101_VERSION_EXPECTED);
  if (version == 0x00 || version == 0xFF) {
    ESP_LOGE(TAG,
             "CC1101 not detected (PARTNUM=0x%02X VERSION=0x%02X) - an all-zero or all-ones reply means "
             "nothing is driving MISO; check wiring and power",
             partnum, version);
    return false;
  }
  if (version != CC1101_VERSION_EXPECTED || partnum != CC1101_PARTNUM_EXPECTED) {
    ESP_LOGW(TAG, "unexpected PARTNUM/VERSION 0x%02X/0x%02X - die revision or clone part, continuing",
             partnum, version);
  }

  this->solve_params_();
  this->write_config_();

  // Read one register back. PARTNUM/VERSION only prove the chip can drive MISO;
  // this proves MOSI reached it too, which is the failure a half-wired module shows.
  const uint8_t sync1 = this->read_reg_(CC1101_REG_SYNC1);
  if (this->spi_error_()) {
    // Raised somewhere between the probe and here, i.e. during write_config_(). The
    // register profile was clocked at a chip that never said it was ready, so some
    // unknown subset of it did not land - and an unknown subset is a radio that may
    // transmit on the wrong frequency or at the wrong deviation. Fail loudly.
    ESP_LOGE(TAG, "CC1101 stopped asserting CHIP_RDYn while the register profile was being written - "
                  "the configuration is incomplete and cannot be trusted");
    return false;
  }
  const uint8_t sync1_expected = (uint8_t) (this->sync_word_ >> 8);
  if (sync1 != sync1_expected) {
    ESP_LOGE(TAG, "register write-back failed: SYNC1 reads 0x%02X, wrote 0x%02X - check MOSI and CS",
             sync1, sync1_expected);
    return false;
  }

  if (!this->go_idle_()) {
    ESP_LOGE(TAG, "CC1101 would not settle in IDLE after configuration (MARCSTATE=0x%02X)", this->marcstate_());
    return false;
  }

  ESP_LOGI(TAG, "CC1101 initialized (TX %.3f MHz, RX %.3f MHz, %.2f kHz dev, %.2f kHz BW, %d dBm, %s)",
           this->freq_hz_ / 1e6f, this->rx_tuned_hz_() / 1e6f, this->deviation_actual_hz_ / 1000.0f,
           this->rx_bandwidth_actual_hz_ / 1000.0f, this->output_power_actual_dbm_,
           (this->modulation_ == Modulation::GFSK) ? "GFSK BT=0.5" : "2-FSK");
  return true;
}

// How long a frame may take to get out, which is the only honest deadline for
// transmit(). The packet engine puts 12 preamble bytes and 2 sync bytes on the air
// ahead of the FIFO data, and every bit of all of it is clocked at the programmed
// 2398.968 baud:
//
//   airtime_ms = (12 + 2 + fifo_bytes) * 8 bits / 2398.968 baud * 1000
//
// `fifo_bytes` is the frame and nothing else: preamble and the whole sync word come
// from the packet engine, so they are the +12 and +2 above and never pass through the
// FIFO. For the 512-byte worst case that is 526 * 8 / 2398.968 = 1754 ms, which is why
// a fixed deadline could not work: it would either be absurd for a 40-byte beacon or
// guaranteed to abort a long frame that was going out perfectly well. The arithmetic
// is integer milli-baud and rounds up, so the computed figure never lands under the
// true airtime.
uint32_t Cc1101Hal::tx_done_timeout_ms_(size_t fifo_bytes) {
  const uint64_t bits = (uint64_t) (CC1101_PREAMBLE_BYTES + CC1101_SYNC_BYTES + fifo_bytes) * 8u;
  const uint32_t airtime_ms =
      (uint32_t) ((bits * 1000000u + CC1101_DRATE_MILLIBAUD - 1u) / CC1101_DRATE_MILLIBAUD);
  return airtime_ms + TX_DONE_MARGIN_MS;
}

// Write `count` bytes of `frame`, starting at `offset`, into the TX FIFO.
//
// Nothing is prepended and nothing is staged: the whole on-air header - preamble and
// all four sync bytes - comes from the packet engine, so the FIFO carries the caller's
// frame and only the caller's frame. Writing it straight through from an offset is
// what lets a 512-byte frame be transmitted with no staging buffer at all, and is
// possible here only because RF433-1 is MSB-first and needs no per-byte transformation
// on the way out.
//
// The bounds clamp is defensive rather than load-bearing - transmit() never asks for a
// slice past the end - but it is the one thing that would read off the caller's buffer
// if the refill arithmetic were ever changed, so it stays.
void Cc1101Hal::write_tx_chunk_(const uint8_t *frame, size_t frame_len, size_t offset, size_t count) {
  if (frame == nullptr || count == 0 || offset >= frame_len) {
    return;
  }
  const size_t left = frame_len - offset;
  this->write_burst_(CC1101_REG_FIFO, frame + offset, (count < left) ? count : left);
}

// ================= TX =================
bool Cc1101Hal::transmit(const uint8_t *frame, size_t len) {
  if (frame == nullptr || len == 0) {
    ESP_LOGW(TAG, "TX: empty frame");
    return false;
  }

  // Every decision below - did TX start, how full is the FIFO, has the packet
  // finished - is read back over SPI, so start from a clean readiness flag. Without
  // this a fault latched by an earlier drain_rx() would be inherited here and blamed
  // on this transmission, and a fault raised here would be invisible to the check
  // before the success return.
  this->clear_spi_error_();
  if (len > TX_FRAME_MAX) {
    // Refused, never truncated. Putting out a prefix would hand the meter a frame
    // whose CRC cannot check out, and the component would then spend its whole
    // receive timeout - and every retry - waiting for an answer that was made
    // impossible here. This ceiling is the component's own frame buffer, so a frame
    // that trips it could not have been built in the first place.
    ESP_LOGE(TAG, "TX: %zu-byte frame exceeds the %zu-byte ceiling - refusing to truncate it", len,
             (size_t) TX_FRAME_MAX);
    return false;
  }

  // NO SYNC WORD IS STRIPPED HERE, and none should be.
  //
  // The sibling RF433-2 backends begin transmit() by stripping a leading 98 F3,
  // because in those components the frame builder puts the sync word at the head of
  // the frame and the packet engine would otherwise emit it twice. RF433-1 does not
  // work that way: RfDataLayer::build_frame() starts the frame at its LENGTH byte
  // (out[RF_TX_LENGTH] = total - 1) and the sync word lives only in the radio's
  // registers - which is also exactly what the RadioHal contract in radio_hal.h
  // promises ("LEN TYPE ... CRCL CRCH on transmit"). There is nothing to strip, and
  // adding a strip "for symmetry" would eat the first two bytes of every frame.
  //
  // Nothing is PREPENDED either. The on-air sync word is 55 55 F6 72, whose leading
  // 55 55 is byte-identical to the preamble and merges with it, so SYNC1/SYNC0 = F6 72
  // and NUM_PREAMBLE = 12 put the complete header on the air from the packet engine.
  // `len` bytes go into the FIFO and `len` bytes come out of the PA. (A previous
  // revision prepended 55 55 here, on the reversed reading of the sync word that
  // fw/RF_PHY_SPEC.md sec. 4.4 prints; see the sync-word note in cc1101_defs.h.)

  if (!this->go_idle_()) {
    ESP_LOGW(TAG, "TX: chip would not go IDLE (MARCSTATE=0x%02X)", this->marcstate_());
    return false;
  }

  // ---- packet length: fixed, or the infinite-to-fixed transition ----
  //
  // PKTLEN is eight bits and RF433-1 frames reach ~292 bytes, so fixed-length mode
  // cannot name every packet this component builds. The standard CC1101 answer is
  // TI's DN500 "infinite-to-fixed" technique, also pointed at by RF_PHY_SPEC.md sec. 7:
  //
  //   1. PKTLEN = len mod 256, and start the packet in INFINITE length mode.
  //   2. Stream bytes. In infinite mode the packet engine never ends the packet, but
  //      its internal length counter runs all the same, modulo 256.
  //   3. Once fewer than 256 bytes remain to be transmitted, switch LENGTH_CONFIG to
  //      FIXED. The counter keeps its value, and the engine now ends the packet when
  //      it reaches PKTLEN - which, because PKTLEN is len mod 256 and we switched
  //      during the final sub-256 stretch, happens after exactly len bytes.
  //
  // The switch is made here when every byte is in the FIFO, which bounds the bytes
  // still to be transmitted by the 64-byte FIFO depth - comfortably inside the 256
  // the technique requires, and the simplest condition that is always both legal and
  // timely. len mod 256 == 0 (len = 256 or 512) is a legitimate PKTLEN of 0 under this
  // scheme, not a zero-length packet: the counter wraps to 0 after the right number
  // of bytes. GDO0 keeps working across the switch, because a packet in infinite mode
  // has no end for GDO0 to fall at until the mode changes.
  //
  // For anything PKTLEN can name on its own, plain fixed-length mode is used from the
  // start - fewer moving parts on the path that carries almost every frame.
  const bool long_packet = (len > TX_PLAIN_FIXED_MAX);
  bool length_fixed = !long_packet;
  if (long_packet) {
    this->write_reg_(CC1101_REG_PKTLEN, (uint8_t) (len & 0xFFu));
    this->write_reg_(CC1101_REG_PKTCTRL0, CC1101_PKTCTRL0_LENGTH_INFINITE);
  } else {
    this->write_reg_(CC1101_REG_PKTCTRL0, CC1101_PKTCTRL0_TX);
    this->write_reg_(CC1101_REG_PKTLEN, (uint8_t) len);
  }

  this->program_frequency_(this->freq_hz_);  // TX sits on the channel's TX carrier, no offset
  this->strobe_(CC1101_STROBE_SFTX);

  // An RF433-1 frame runs well past the 64-byte FIFO, so the FIFO is primed with what
  // fits and refilled as it drains - writing the whole frame up front would simply
  // drop the tail, exactly as it would on the CMT2300A.
  size_t written = (len > CC1101_FIFO_SIZE) ? CC1101_FIFO_SIZE : len;
  this->write_tx_chunk_(frame, len, 0, written);

  this->strobe_(CC1101_STROBE_STX);

  // `written` only ever means "loaded into the TX FIFO", never "sent on air", so it
  // cannot by itself distinguish a completed packet from an STX that was rejected or
  // ineffective and left the chip sitting in IDLE with a full FIFO. Both look like
  // "everything written and MARCSTATE == IDLE". Without positive evidence that the
  // chip really entered TX, the second case is reported as a successful transmission
  // and the component then burns its entire receive timeout waiting for a reply to a
  // request that never went out. So TX entry is tracked as a fact in its own right:
  // observed here on entry, or later in the loop from MARCSTATE, or from GDO0
  // asserting, which under IOCFG0 = 0x06 means the sync word has physically gone out.
  bool tx_started = this->wait_marcstate_(CC1101_MARC_TX, TX_ENTER_TIMEOUT_MS);
  if (!tx_started) {
    // Tell "STX did nothing" apart from "we looked too slowly". The shortest frame
    // here is ~47 ms of air, so the packet cannot have come and gone inside the 20 ms
    // wait; if the chip is still in IDLE with every loaded byte still in the FIFO,
    // nothing has gone out and nothing will, and waiting out the full packet deadline
    // would only delay the retry.
    const uint8_t marc = this->marcstate_();
    const uint8_t in_fifo = this->read_status_reg_(CC1101_REG_TXBYTES) & CC1101_FIFO_COUNT_MASK;
    if (marc == CC1101_MARC_IDLE && in_fifo >= written) {
      ESP_LOGW(TAG, "TX: STX did not start a transmission (MARCSTATE=0x%02X, %u/%zu bytes still in the "
                    "FIFO) - nothing went on the air",
               marc, (unsigned) in_fifo, written);
      this->strobe_(CC1101_STROBE_SIDLE);
      this->strobe_(CC1101_STROBE_SFTX);
      this->go_idle_();
      return false;
    }
    ESP_LOGW(TAG, "TX: entry not confirmed within %" PRIu32 " ms (MARCSTATE=0x%02X) - watching for a late "
                  "start",
             TX_ENTER_TIMEOUT_MS, marc);
  }

  const bool chunked = len > CC1101_FIFO_SIZE;
  // Derived from this frame's own airtime rather than a one-size constant - see
  // tx_done_timeout_ms_(), which adds the preamble and sync bytes the packet engine
  // contributes on top of the FIFO byte count.
  const uint32_t done_timeout_ms = tx_done_timeout_ms_(len);
  const uint32_t t0 = esphome::millis();
  bool sync_seen = false;
  bool done = false;
  while (esphome::millis() - t0 < done_timeout_ms) {
    const uint8_t txbytes = this->read_status_reg_(CC1101_REG_TXBYTES);
    if ((txbytes & CC1101_TXBYTES_UNDERFLOW) != 0) {
      // The PA ran out of bytes mid-packet: whatever reached the air is truncated
      // and the chip is wedged in TXFIFO_UNDERFLOW until the FIFO is flushed.
      ESP_LOGW(TAG, "TX FIFO underflow after %zu/%zu bytes - frame truncated on air", written, len);
      this->strobe_(CC1101_STROBE_SIDLE);
      this->strobe_(CC1101_STROBE_SFTX);
      this->go_idle_();
      return false;
    }

    const uint8_t in_fifo = txbytes & CC1101_FIFO_COUNT_MASK;
    if (written < len && in_fifo <= TX_REFILL_LOW_WATER) {
      // Refill to the brim once the FIFO has drained past the threshold programmed
      // in FIFOTHR (33 bytes in TX). At 2400 baud a full FIFO is 213 ms of air, so
      // the loop has an enormous margin over the poll interval.
      const size_t room = CC1101_FIFO_SIZE - in_fifo;
      const size_t remaining = len - written;
      const size_t chunk = (remaining > room) ? room : remaining;
      this->write_tx_chunk_(frame, len, written, chunk);
      written += chunk;
    }

    // The infinite-to-fixed switch, step 3 above. Everything is in the FIFO, so at
    // most CC1101_FIFO_SIZE bytes remain to be transmitted - well inside the 256 the
    // technique needs - and PKTLEN (= len mod 256) now names where the packet ends.
    if (!length_fixed && written >= len) {
      this->write_reg_(CC1101_REG_PKTCTRL0, CC1101_PKTCTRL0_LENGTH_FIXED);
      length_fixed = true;
      ESP_LOGVV(TAG, "TX: switched to fixed length with %u byte(s) left in the FIFO (PKTLEN=%u)",
                (unsigned) in_fifo, (unsigned) (len & 0xFFu));
    }

    if (this->gdo0_pin_ != nullptr) {
      // IOCFG0 = 0x06: GDO0 rises when the sync word has gone out and falls at the
      // end of the packet. That falling edge is the exact TX-done marker; without
      // the pin the MARCSTATE check below gets there a poll or two later. A high
      // GDO0 is also the strongest possible evidence that TX started - the sync word
      // is already on the air - so it settles tx_started on its own.
      if (pin_read(this->gdo0_)) {
        sync_seen = true;
        tx_started = true;
      } else if (sync_seen) {
        done = true;
        break;
      }
    }

    // One MARCSTATE read serves both purposes: it is the late-start evidence when
    // entry was missed above, and the completion test once everything is written.
    const uint8_t marc = this->marcstate_();
    if (marc == CC1101_MARC_TX || marc == CC1101_MARC_TX_END) {
      tx_started = true;
    }
    // MCSM1.TXOFF_MODE = 00 parks the chip in IDLE when the packet is done, so
    // "everything written and back in IDLE" is an unambiguous completion - but only
    // once TX is known to have started, because an unstarted TX sits in IDLE too and
    // would otherwise satisfy this on the very first poll.
    if (tx_started && written >= len && marc == CC1101_MARC_IDLE) {
      done = true;
      break;
    }

    esphome::delayMicroseconds(200);
    esphome::yield();
  }

  const uint32_t dur = esphome::millis() - t0;
  if (!tx_started) {
    // The deadline passed without the chip ever being seen in TX or TX_END and
    // without GDO0 ever asserting. Whatever is in the FIFO was never modulated, so
    // this is a failed transmission and must be reported as one: a "success" here
    // would send the component into a receive epoch that cannot possibly produce a
    // reply. Flush the FIFO on the way out so the next attempt starts empty.
    ESP_LOGW(TAG, "TX never started: no TX state and no GDO0 sync in %" PRIu32 " ms (MARCSTATE=0x%02X) - "
                  "the frame did not go on the air",
             dur, this->marcstate_());
    this->strobe_(CC1101_STROBE_SIDLE);
    this->strobe_(CC1101_STROBE_SFTX);
    this->go_idle_();
    return false;
  }
  if (!done) {
    ESP_LOGW(TAG, "TX done timeout after %" PRIu32 " ms (%zu/%zu written, MARCSTATE=0x%02X TXBYTES=0x%02X)",
             dur, written, len, this->marcstate_(), this->read_status_reg_(CC1101_REG_TXBYTES));
    this->go_idle_();
    return false;
  }
  if (written < len) {
    // The packet ended with bytes still unsent, so what reached the air is a frame
    // whose CRC cannot check out and which the meter will never answer. That is a
    // failed transmission, not a qualified success: say so, and let the component
    // resend rather than spend an RX timeout waiting on it. (The CMT2300A backend
    // reports the same condition the same way - the two must not disagree about
    // what "transmit() returned true" means.)
    ESP_LOGW(TAG, "TX complete but only %zu/%zu written - frame truncated on air", written, len);
    this->go_idle_();
    return false;
  }
  if (this->spi_error_()) {
    // CHIP_RDYn dropped out at some point during the transmission, so every status
    // read that got us here - the TX-entry observation, the TXBYTES refill accounting,
    // the return to IDLE that was read as "packet sent" - came off a chip that had not
    // said it was ready. None of it is evidence, and the one conclusion that must not
    // be drawn from non-evidence is "the frame went out": a false success sends the
    // component into a receive epoch for a request that may never have been
    // modulated, costing it the whole receive timeout. Report the failure and let the
    // caller resend.
    ESP_LOGW(TAG, "TX: CC1101 stopped asserting CHIP_RDYn during the transmission - completion cannot be "
                  "confirmed, treating the frame as not sent");
    this->strobe_(CC1101_STROBE_SIDLE);
    this->strobe_(CC1101_STROBE_SFTX);
    this->go_idle_();
    return false;
  }
  // For a chunked (> FIFO) frame, log timing and fill so a clean send (dur ~ airtime,
  // written == len) can be told apart from a FIFO underrun (dur far too short).
  if (chunked) {
    ESP_LOGVV(TAG, "TX done (chunked): %zu/%zu written, %" PRIu32 " ms", written, len, dur);
  }
  this->go_idle_();
  return true;
}

// ================= RX =================
// This is the whole begin_rx() epoch contract from radio_hal.h: SIDLE aborts a TX or
// RX still in flight and is also the only way out of a latched RXFIFO_OVERFLOW,
// SFRX empties the FIFO so not one byte of a late or abandoned reply can be read as
// the head of the next one, and SRX is confirmed in MARCSTATE before returning true.
// There is no interrupt-flag register to clear on this chip - the GDO lines are
// level outputs of the current state, not latches - and the driver keeps no software
// receive buffer of its own, so the FIFO flush plus the sync-tail reset below is the
// complete flush.
bool Cc1101Hal::begin_rx() {
  // Clear FIRST, before anything can fail. Every early return below leaves the epoch
  // disarmed, which is what stops drain_rx() handing the caller bytes that belong to
  // no epoch: if go_idle_() fails, SFRX is never reached, so the FIFO still holds
  // whatever the last reception left in it and the chip may still be sitting in RX
  // from before - which is exactly the shape drain_rx() would otherwise read as a
  // healthy "DATA". The component does not escalate a false return from here; it
  // relies on the next drain_rx() saying NOT_IN_RX, so that has to be true.
  this->rx_epoch_armed_ = false;
  // Every confirmation below is an SPI read, so start from a clean readiness flag.
  this->clear_spi_error_();

  if (!this->go_idle_()) {
    ESP_LOGW(TAG, "RX: chip would not go IDLE (MARCSTATE=0x%02X)", this->marcstate_());
    return false;
  }

  // RX is an independent carrier, not an offset from TX: in alternative-channel mode
  // the CIU keeps transmitting on 433.82 MHz while the meter replies on the
  // advertised channel's carrier. set_rx_frequency() supplies that carrier and the
  // YAML rx_offset nudges it; no IF term is applied, because this part is not a
  // low-IF receiver (see freq_word_for_()).
  this->program_frequency_(this->rx_tuned_hz_());

  // Infinite packet length. The chip's variable-length engine reads the first byte
  // after sync as the length, and RF433-1's is total_length - 1 - off by one, and in
  // any case the frames here outgrow an 8-bit field. The component bounds the frame
  // by that byte itself (see drain_rx in radio_hal.h), so all the packet engine has
  // to do is find the sync word and never stop. PKTLEN is restored to its resting
  // value because transmit() leaves it at the last TX length and it would otherwise
  // be wrong the moment the mode changed.
  this->write_reg_(CC1101_REG_PKTCTRL0, CC1101_PKTCTRL0_RX);
  this->write_reg_(CC1101_REG_PKTLEN, 0xFF);

  // Nothing has to be armed for the sync word here. The correlator matches F6 72,
  // the 55 55 ahead of it is preamble as far as the chip is concerned, and the first
  // byte the FIFO delivers after sync detection is already the frame's LEN. See the
  // sync-word note in cc1101_defs.h.
  this->strobe_(CC1101_STROBE_SFRX);
  this->strobe_(CC1101_STROBE_SRX);
  if (!this->wait_marcstate_(CC1101_MARC_RX, STATE_TIMEOUT_MS)) {
    ESP_LOGW(TAG, "RX: no RX state (MARCSTATE=0x%02X)", this->marcstate_());
    return false;
  }
  if (this->spi_error_()) {
    // CHIP_RDYn dropped out somewhere in the sequence above, so the MARCSTATE that
    // just read back as RX came off a chip that had not said it was ready. A value
    // that only looks like RX is not confirmation that RX was entered, and the whole
    // point of this function is to return positive evidence. Leave the epoch
    // disarmed and fail.
    ESP_LOGW(TAG, "RX: CC1101 stopped asserting CHIP_RDYn while entering RX - the state is unknown");
    return false;
  }
  this->rx_epoch_armed_ = true;
  return true;
}

// MARCSTATE values that mean the receive epoch is over rather than merely between
// states. Entering RX walks through CALIBRATE, SETTLING and FS_LOCK, and RX_END is
// transient, so none of those is a fault; what is a fault is the chip sitting
// somewhere it cannot leave by itself - IDLE or SLEEP, meaning something took it out
// of RX, or any TX-side state, meaning it is not listening at all. RXFIFO_OVERFLOW
// is deliberately absent: drain_rx() handles and recovers that one itself.
static bool marcstate_is_dead_rx(uint8_t marc) {
  switch (marc) {
    case CC1101_MARC_SLEEP:
    case CC1101_MARC_IDLE:
    case CC1101_MARC_XOFF:
    case CC1101_MARC_FSTXON:
    case CC1101_MARC_TX:
    case CC1101_MARC_TX_END:
    case CC1101_MARC_TXFIFO_UNDERFLOW:
      return true;
    default:
      return false;
  }
}

size_t Cc1101Hal::drain_rx(uint8_t *buf, size_t buf_size, RxStatus &status) {
  status = RxStatus::RX_STATUS_NO_DATA;
  if (buf == nullptr || buf_size == 0) {
    return 0;
  }

  if (!this->rx_epoch_armed_) {
    // begin_rx() never confirmed an epoch, or something has since taken the chip out
    // of RX through go_idle_(). Either way the FIFO may still hold bytes from an
    // earlier reception, and handing those over would be the exact desync the
    // begin_rx() contract in radio_hal.h exists to prevent: a late reply to a request
    // that already timed out, drained as the head of the NEXT reply. Say the epoch is
    // dead instead - which is also what the component is relying on, since it does
    // not escalate a false return from begin_rx() itself.
    status = RxStatus::RX_STATUS_NOT_IN_RX;
    return 0;
  }

  // Everything below is read over SPI, so start from a clean readiness flag: a
  // CHIP_RDYn timeout during this poll must surface as IO_ERROR rather than being
  // inherited from, or leaked into, another one.
  this->clear_spi_error_();

  const uint8_t rxbytes = this->read_status_reg_(CC1101_REG_RXBYTES);
  if (this->spi_error_()) {
    // The chip never signalled ready, so this byte came out of a shift register that
    // was not driven by a running SPI interface. It is not a FIFO count and must not
    // be used as one - acting on it would read an arbitrary number of bytes out of
    // the FIFO. IO_ERROR ends the epoch, which is the right answer: a radio that
    // will not answer CHIP_RDYn is not receiving either.
    ESP_LOGW(TAG, "RX: CC1101 did not assert CHIP_RDYn within %" PRIu32 " us - the SPI read is meaningless",
             CHIP_RDY_TIMEOUT_US);
    status = RxStatus::RX_STATUS_IO_ERROR;
    return 0;
  }
  if ((rxbytes & CC1101_RXBYTES_OVERFLOW) != 0) {
    // The chip stops receiving the moment the FIFO overflows and stays wedged in
    // RXFIFO_OVERFLOW until it is flushed from IDLE. Whatever is in there is a
    // truncated frame at best, so drop it and get back to listening. The caller is
    // told OVERFLOW rather than "no bytes": the reply is gone, and waiting out the
    // RX timeout for it would only delay the retry that might still succeed.
    ESP_LOGW(TAG, "RX FIFO overflow - flushing and restarting RX");
    this->strobe_(CC1101_STROBE_SIDLE);
    this->strobe_(CC1101_STROBE_SFRX);
    this->strobe_(CC1101_STROBE_SRX);
    status = RxStatus::RX_STATUS_OVERFLOW;
    return 0;
  }

  const uint8_t available = rxbytes & CC1101_FIFO_COUNT_MASK;
  if (available > CC1101_RXBYTES_MAX_VALID) {
    // 65 is a REAL count, not a fault: the receive chain holds one byte in a prefetch
    // register ahead of the 64-byte FIFO and NUM_RXBYTES counts it, which is why the
    // bound here is CC1101_RXBYTES_MAX_VALID (= 65) and not the FIFO depth. Rejecting
    // 65 is actively harmful - a loop() that ran late lets the FIFO reach exactly that
    // while the reply is still arriving, and calling it a bus failure throws away a
    // frame that was being received correctly. Only a count ABOVE 65 is a value the
    // part cannot produce; read_status_reg_() already re-reads until two agree, so two
    // reads agreeing on something impossible means the bus is wrong - a floating MISO
    // or a wire that has come off - not the documented errata race.
    ESP_LOGW(TAG, "RX: implausible RXBYTES 0x%02X (above the %u-byte FIFO-plus-prefetch maximum) - the SPI "
                  "read cannot be trusted",
             rxbytes, (unsigned) CC1101_RXBYTES_MAX_VALID);
    status = RxStatus::RX_STATUS_IO_ERROR;
    return 0;
  }

  const uint8_t marc = this->marcstate_();
  if (this->spi_error_()) {
    // CHIP_RDYn went away between the RXBYTES read and this one, so `marc` came out of
    // a chip that had not said it was ready and is not a state reading. It must not be
    // judged: marcstate_is_dead_rx() on a meaningless byte either invents a dead epoch
    // or, worse, hides a real one. Reporting IO_ERROR instead is also what keeps the
    // sticky flag from being silently dropped - the next drain_rx() clears it, so a
    // fault observed here and not acted on here is a fault that never happened, and
    // the component would wait out its whole receive timeout for a radio that is not
    // answering at all.
    ESP_LOGW(TAG, "RX: CC1101 stopped asserting CHIP_RDYn while reading MARCSTATE - the state is unknown");
    status = RxStatus::RX_STATUS_IO_ERROR;
    return 0;
  }
  if (available == 0) {
    if (marcstate_is_dead_rx(marc)) {
      ESP_LOGW(TAG, "RX: chip left RX with an empty FIFO (MARCSTATE=0x%02X)", marc);
      status = RxStatus::RX_STATUS_NOT_IN_RX;
    }
    return 0;
  }

  // Documented CC1101 FIFO hazard (errata SWRZ020E): emptying the RX FIFO completely
  // while the demodulator is still writing into it can return a byte that was never
  // received, and the errata's own workaround is to read n - 1 bytes. While the chip
  // is in RX we therefore always leave the last byte behind - including at the 65
  // the prefetch register makes possible, where n - 1 is 64 and the byte left behind
  // is the prefetched one. Nothing is stranded by this: infinite-length mode keeps
  // demodulating noise after the reply, so more bytes always arrive behind the one
  // held back - and the component's drain-stall fallback calls read_rx_exact() if
  // they somehow do not.
  const bool receiving = (marc == CC1101_MARC_RX);
  size_t take = receiving ? (size_t) (available - 1u) : (size_t) available;
  if (take > CC1101_FIFO_SIZE) {
    // Only reachable at available == 65 with the chip already out of RX. Capping at
    // the FIFO depth keeps max_drain_chunk() an exact bound rather than an estimate,
    // which is what the component's "buffer is full" fallback reasons from; the odd
    // byte is not lost, the next poll takes it (and the FIFO is no longer filling, so
    // there is no race to lose it to).
    take = CC1101_FIFO_SIZE;
  }
  if (take == 0) {
    return 0;
  }
  if (take > buf_size) {
    take = buf_size;
  }

  this->read_fifo_burst_(buf, take);
  if (this->spi_error_()) {
    // CHIP_RDYn dropped out between the count read and the burst, so the bytes just
    // clocked in are not the FIFO's contents. Appending them would feed the frame
    // parser garbage dressed up as a reply, which is worse than reporting nothing.
    ESP_LOGW(TAG, "RX: CC1101 stopped asserting CHIP_RDYn during the FIFO burst - discarding %zu bytes",
             take);
    status = RxStatus::RX_STATUS_IO_ERROR;
    return 0;
  }

  // NO BIT REVERSAL. The sibling RF433-2 backends reverse8() every byte here because
  // that PHY is LSB-first on air; RF433-1 is MSB-first, which is the order the CC1101
  // already shifts in, so the bytes are handed on exactly as received. Nothing is
  // stripped off the front either - the sync word is consumed by the correlator and
  // never reaches the FIFO, so the first byte of this epoch is the frame's own LEN.

  // Bytes in hand are reported as DATA even if the chip has since left RX: they were
  // received, and the next poll is where the dead epoch gets noticed.
  status = RxStatus::RX_STATUS_DATA;
  return take;
}

// Read up to `len` bytes out of the RX FIFO with no hold-back - see the contract note
// in cc1101_hal.h for why this one does NOT apply the errata's n - 1 rule.
//
// Every byte the FIFO holds in a receive epoch belongs to the caller: the sync word is
// consumed by the correlator and never reaches the FIFO, so there is nothing to skip
// and no scratch buffer is needed.
size_t Cc1101Hal::read_rx_exact(uint8_t *buf, size_t len) {
  if (buf == nullptr || len == 0) {
    return 0;
  }
  this->clear_spi_error_();

  const uint8_t rxbytes = this->read_status_reg_(CC1101_REG_RXBYTES);
  if (this->spi_error_()) {
    ESP_LOGW(TAG, "RX: CC1101 did not assert CHIP_RDYn while sizing a direct FIFO read");
    return 0;
  }
  if ((rxbytes & CC1101_RXBYTES_OVERFLOW) != 0) {
    // Recovery belongs to drain_rx(), which also has a status to report it through.
    // This method can only return a count, so it returns none and lets the next poll
    // surface the overflow properly.
    ESP_LOGW(TAG, "RX: direct FIFO read found the FIFO overflowed - leaving recovery to the next drain");
    return 0;
  }
  const size_t available = rxbytes & CC1101_FIFO_COUNT_MASK;
  if (available > CC1101_RXBYTES_MAX_VALID) {
    ESP_LOGW(TAG, "RX: implausible RXBYTES 0x%02X during a direct FIFO read - ignoring it", rxbytes);
    return 0;
  }

  const size_t take = (len < available) ? len : available;
  if (take == 0) {
    return 0;
  }
  this->read_fifo_burst_(buf, take);
  if (this->spi_error_()) {
    ESP_LOGW(TAG, "RX: CC1101 stopped asserting CHIP_RDYn during a direct FIFO read - discarding %zu bytes",
             take);
    return 0;
  }
  return take;
}

// Datasheet section 17.3: the RSSI register is a 2's complement value in half-dB
// steps, offset by 74 dB at 433 MHz. Free-running, which is why set_rssi_mode() is
// left at the RadioHal no-op default - the CMT2300A needs a mode switch here because
// it multiplexes RSSI reporting and FIFO merging through one register field, and this
// part does not.
int8_t Cc1101Hal::get_rssi_dbm() {
  this->clear_spi_error_();
  const int rssi_dec = (int) this->read_status_reg_(CC1101_REG_RSSI);
  const int dbm = (rssi_dec >= 128) ? (((rssi_dec - 256) / 2) - 74) : ((rssi_dec / 2) - 74);
  if (this->spi_error_()) {
    // The byte came off a chip that had not asserted CHIP_RDYn, so it is not an RSSI
    // reading. It is still returned, because RadioHal::get_rssi_dbm() has no way to
    // say "no value" - the signature is shared with the two sibling components and
    // widening it is not this change's business - but it is logged, because the
    // channel survey feeds this number into the quality nibble it advertises on air
    // and a silent garbage sample there is invisible afterwards. If this line ever
    // appears in a log, the right fix is to give the interface a validity channel.
    ESP_LOGW(TAG, "RSSI: CC1101 did not assert CHIP_RDYn - %d dBm is not a measurement", dbm);
  }
  return (int8_t) dbm;
}

// Called from the component's dump_config() after its own lines, so the two-space
// indent makes these read as part of the same block. Everything printed here is the
// value the chip was actually programmed with rather than the value asked for in
// YAML, so the log is evidence of what went on the air.
void Cc1101Hal::dump_config() {
  LOG_PIN("  SCLK pin: ", this->sclk_pin_);
  LOG_PIN("  MISO pin: ", this->miso_pin_);
  LOG_PIN("  MOSI pin: ", this->mosi_pin_);
  LOG_PIN("  CS pin: ", this->cs_pin_);
  if (this->gdo0_pin_ != nullptr) {
    LOG_PIN("  GDO0 pin: ", this->gdo0_pin_);
  } else {
    ESP_LOGCONFIG(TAG, "  GDO0 pin: not used (TX completion polled from MARCSTATE)");
  }

  const uint32_t tx_word = freq_word_for_(this->freq_hz_);
  const uint32_t tx_hz = freq_hz_for_word_(tx_word);
  const uint32_t rx_requested = this->rx_tuned_hz_();
  const uint32_t rx_word = freq_word_for_(rx_requested);
  const uint32_t rx_hz = freq_hz_for_word_(rx_word);
  const float data_rate =
      (256.0f + (float) CC1101_DRATE_M) * (float) (1u << CC1101_DRATE_E) * (float) CC1101_XTAL_HZ / 268435456.0f;

  // TX and RX are printed separately and in full because on this protocol they are
  // genuinely different carriers in alternative-channel mode - a reader checking a
  // "why does the meter not answer" log needs to see both, not one and an offset.
  ESP_LOGCONFIG(TAG, "  TX carrier: %.4f MHz (FREQ=0x%06" PRIX32 ", %+d Hz off the %.3f MHz channel)",
                tx_hz / 1e6f, tx_word, (int) ((int32_t) tx_hz - (int32_t) this->freq_hz_),
                this->freq_hz_ / 1e6f);
  ESP_LOGCONFIG(TAG, "  RX carrier: %.4f MHz (FREQ=0x%06" PRIX32 ", %.3f MHz channel %+" PRId32 " Hz offset)",
                rx_hz / 1e6f, rx_word, this->rx_freq_hz_ / 1e6f, this->rx_offset_hz_);
  ESP_LOGCONFIG(TAG, "  Deviation: %.2f kHz (DEVIATN=0x%02X, DEV_E=%u DEV_M=%u)",
                this->deviation_actual_hz_ / 1000.0f, (unsigned) ((this->dev_e_ << 4) | this->dev_m_),
                (unsigned) this->dev_e_, (unsigned) this->dev_m_);
  // TEST2/TEST1 are printed next to the bandwidth they were chosen for, because that
  // is the only way to see at a glance that the pair matches the filter: 0x81/0x35 at
  // or below 101 kHz, 0x88/0x31 above it.
  ESP_LOGCONFIG(TAG, "  RX bandwidth: %.2f kHz (CHANBW_E=%u CHANBW_M=%u, TEST2=0x%02X TEST1=0x%02X)",
                this->rx_bandwidth_actual_hz_ / 1000.0f, (unsigned) this->chanbw_e_,
                (unsigned) this->chanbw_m_, this->test2_, this->test1_);
  // The FOC/bit-sync carrier-sense gate is the bring-up knob most likely to need
  // changing on real hardware, so the effective register value is on every boot log:
  // 0x16 means both loops track from the moment RX is entered, 0x36 means they stay
  // frozen until carrier sense asserts. See set_foc_cs_gate().
  ESP_LOGCONFIG(TAG, "  FOC/bit-sync carrier-sense gate: %s (FOCCFG=0x%02X)",
                this->foc_cs_gate_ ? "ON - loops frozen until carrier sense"
                                   : "OFF - loops track continuously",
                this->foccfg_value_());
  ESP_LOGCONFIG(TAG, "  Data rate: %.2f baud (DRATE_E=%u DRATE_M=%u), %s, MSB-first",
                data_rate, (unsigned) CC1101_DRATE_E, (unsigned) CC1101_DRATE_M,
                (this->modulation_ == Modulation::GFSK) ? "GFSK BT=0.5 (the real CIU's shaping)"
                                                        : "2-FSK (the spec's reference)");
  // Printed in full because it is the one place the 16-bit sync register and the
  // 4-byte on-air sync word are reconciled, and because the byte order is derived
  // rather than SDR-confirmed - a reader chasing "the meter never answers" should be
  // able to read what actually went out without rebuilding. The leading 55 55 is the
  // preamble's last two bytes as far as the chip is concerned, which is why 12
  // preamble bytes plus a 2-byte correlator reproduce the CIU's 10 + 4 exactly.
  ESP_LOGCONFIG(TAG, "  Sync: 55 55 %02X %02X on air - %02X %02X in SYNC1/SYNC0 (16/16 exact), the "
                     "leading 55 55 merged into %u preamble bytes",
                (uint8_t) (this->sync_word_ >> 8), (uint8_t) (this->sync_word_ & 0xFFu),
                (uint8_t) (this->sync_word_ >> 8), (uint8_t) (this->sync_word_ & 0xFFu),
                (unsigned) CC1101_PREAMBLE_BYTES);
  // The deficit is worth printing on every boot: it is a standing candidate
  // explanation for a meter that hears a real CIU but not us. It is a far smaller gap
  // than the sibling RF433-2 projects face, so it is stated as a number rather than
  // as a warning.
  ESP_LOGCONFIG(TAG, "  Output power: %d dBm (PATABLE[0]=0x%02X) - a Nartis D101 CIU transmits %d dBm, so "
                     "the uplink is %d dB short of the reference link budget",
                this->output_power_actual_dbm_, this->pa_value_, CC1101_CIU_REFERENCE_POWER_DBM,
                CC1101_CIU_REFERENCE_POWER_DBM - this->output_power_actual_dbm_);
}

}  // namespace esphome::nartis_rf_meter
