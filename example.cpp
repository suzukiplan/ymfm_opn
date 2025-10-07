#include "ymfm_ym2612.hpp"

class VgmDriver : public ymfm::ymfm_interface
{
  private:
    ymfm::ym2612 fm;

  public:
    VgmDriver() : fm(*this)
    {
    }

    ~VgmDriver()
    {
    }

    void reset() { this->fm.reset(); }

    //
    // timing and synchronizaton
    //

    // the chip implementation calls this when a write happens to the mode
    // register, which could affect timers and interrupts; our responsibility
    // is to ensure the system is up to date before calling the engine's
    // engine_mode_write() method
    virtual void ymfm_sync_mode_write(uint8_t data) { m_engine->engine_mode_write(data); }

    // the chip implementation calls this when the chip's status has changed,
    // which may affect the interrupt state; our responsibility is to ensure
    // the system is up to date before calling the engine's
    // engine_check_interrupts() method
    virtual void ymfm_sync_check_interrupts() { m_engine->engine_check_interrupts(); }

    // the chip implementation calls this when one of the two internal timers
    // has changed state; our responsibility is to arrange to call the engine's
    // engine_timer_expired() method after the provided number of clocks; if
    // duration_in_clocks is negative, we should cancel any outstanding timers
    virtual void ymfm_set_timer(uint32_t tnum, int32_t duration_in_clocks) {}

    // the chip implementation calls this to indicate that the chip should be
    // considered in a busy state until the given number of clocks has passed;
    // our responsibility is to compute and remember the ending time based on
    // the chip's clock for later checking
    virtual void ymfm_set_busy_end(uint32_t clocks) {}

    // the chip implementation calls this to see if the chip is still currently
    // is a busy state, as specified by a previous call to ymfm_set_busy_end();
    // our responsibility is to compare the current time against the previously
    // noted busy end time and return true if we haven't yet passed it
    virtual bool ymfm_is_busy() { return false; }
};

int main(int argc, char* argv[])
{
    VgmDriver vgmdrv;
    vgmdrv.reset();
    return 0;
}
