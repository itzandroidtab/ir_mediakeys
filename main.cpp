#include <atomic>
#include <expected>

#include <klib/klib.hpp>
#include <klib/delay.hpp>
#include <klib/ringbuffer.hpp>
#include <klib/usb/device/keyboard.hpp>

#include <io/timer.hpp>
#include <io/watchdog.hpp>
#include <io/pins.hpp>
#include <io/usb.hpp>
#include <io/system.hpp>

namespace target = klib::target;

enum class ir_key: uint32_t {
    up = 0x77e1d03a,
    down = 0x77e1b03a,
    right = 0x77e1e03a,
    left = 0x77e1103a,
    menu = 0x77e1403a,
    play = 0x77e1203a,

    // special codes
    released = 0x7e1603a,
    repeat = 0xffffffff,
    none = 0x00,
};  

// create the usb with keboard hid driver
using usb = target::io::usb<
    target::io::periph::wlcsp_64::usb0,
    klib::usb::device::keyboard_hid<1>
>;

/**
 * @brief Helper function to convert an ir key to a usb consumer key
 * 
 * @param key 
 * @return std::expected<usb::device::consumer_key_t, bool> 
 */
std::expected<usb::device::consumer_key_t, bool> ir_key_to_usb(ir_key key) {
    // convert the ir key to a usb key
    switch (key) {
        case ir_key::up:
            return usb::device::consumer_key_t::key_volume_up;
        case ir_key::down:
            return usb::device::consumer_key_t::key_volume_down;
        case ir_key::right:
            return usb::device::consumer_key_t::key_media_next_track;
        case ir_key::left:
            return usb::device::consumer_key_t::key_media_previous_track;
        case ir_key::menu:
            return usb::device::consumer_key_t::key_mute;
        case ir_key::play:
            return usb::device::consumer_key_t::key_media_play_pause;
        default:
            return std::unexpected(false);
    }
}

/**
 * @brief Very basic ir receiver using a pin interrupt and two one 
 * shot timers. Does not bother to check exact timing. Only uses falling
 * interrupts to detect the bits. Timer is used to determine if we have
 * a logical 1 or 0 and a timeout to determine the end of the message.
 * 
 * @tparam Pin 
 * @tparam BitTimer 
 * @tparam TimeoutTimer 
 * @tparam Size 
 */
template <typename Pin, typename BitTimer, typename TimeoutTimer, uint32_t Size = 16>
class ir_receiver {
private:
    // ringbuffer to store the received codes
    static inline klib::ringbuffer<uint32_t, Size> ir_buffer;

    // flag to indicate if we have received data. Used to prevent
    // compiler optimizations to remove the ir_buffer checks
    static inline volatile bool has_data_received;

    // raw payload data. Used for storing the bits as they come in
    static inline std::atomic<uint32_t> payload;

    // amount of bits received
    static inline std::atomic<uint8_t> bits;

    // flag to indicate if we have received the sequence
    static inline std::atomic<bool> start_received;

    /**
     * @brief Timeout callback when no edge was detected for a while
     * 
     */
    static void timeout_callback() {
        // we have a timeout. This means the message is done
        // if we have received a start sequence and a single bit
        // it is a repeat code
        if (start_received && (bits <= 1)) {
            // push the repeat code
            ir_buffer.push(0xffffffff);
            has_data_received = true;
        }
        else if (bits >= 32) {
            // we have received a full payload
            ir_buffer.push(payload);
            has_data_received = true;
        }

        // reset the state
        payload = 0;
        bits = 0;
        start_received = false;
    }

    /**
     * @brief Callback when a falling edge is detected
     * 
     */
    static void edge_callback() {
        // check if the bit timer was done. If that happened we have a 
        // logical 1. This is only used when we have already received
        // the start sequence
        const bool logical_one = BitTimer::done();

        // we have received a falling edge. Start the bit and timeout timers
        BitTimer::init(nullptr, 1'000'000 / 1'500);
        TimeoutTimer::init(timeout_callback, 1'000'000 / 15'000);

        // start the timers
        BitTimer::enable();
        TimeoutTimer::enable();

        // check if we have already received the start sequence
        if (!start_received) {
            start_received = true;
            return;
        }

        // shift our data into the temporary payload
        payload = (payload << 1) | logical_one;
        bits++;
    }

public:
    /**
     * @brief Initialize the ir receiver
     * 
     */
    static void init() {
        // initialize the pin using a falling edge
        Pin::template init<Pin::edge::falling>(edge_callback);
    }

    /**
     * @brief Return whether the ringbuffer is empty.
     *
     * @return
     */
    static bool has_data() {
        return has_data_received;
    }

    /**
     * @brief Return the next value from the buffer.
     * 
     * @return uint32_t 
     */
    static uint32_t read() {
        const auto ret = ir_buffer.copy_and_pop();

        if (ir_buffer.empty()) {
            has_data_received = false;
        }

        return ret;
    }
};

int main() {
    // disable the watchdog
    target::io::watchdog<target::io::periph::wdt0>::disable();

    // get the pin for the ir receiver
    using ir_pin = target::pins::package::wlcsp_64::pe7;

    // create 2 timers. Note we need dividers here to limit the range to 16 bit
    using timer0 = target::io::oneshot_timer<target::io::periph::wlcsp_64::tc0, 0, 32>;
    using timer1 = target::io::oneshot_timer<target::io::periph::wlcsp_64::tc0, 1, 32>;

    // create a ir pin uisng the pin_interrupt
    using pin = target::io::pin_interrupt<ir_pin>;

    // create and init the ir receiver
    ir_receiver<pin, timer0, timer1> receiver;
    receiver.init();

    // set the priority of the port interrupt to be higher than the systick
    target::interrupt_priority<ir_pin::port::interrupt_id, 1>();

    // setup the usb clock using the external clock
    target::io::system::clock::set_usb<
        klib::core::atsam4s::io::system::clock::pll::pllb,
        12'000'000, 16, 2, 2
    >();

    usb::init();

    // wait until we are configured. (this happens after the host has connected)
    while (!usb::device::is_configured<usb>()) {
        klib::delay(klib::time::ms(10));
    }

    ir_key previous_key = ir_key::none;

    // send all the keypresses to the usb host
    while (true) {
        // check if we have data
        if (!receiver.has_data()) {
            // no data yet. wait a bit
            klib::delay(klib::time::ms(1));

            // continue the main loop
            continue;
        }

        // read the key from the receiver
        const ir_key key = static_cast<ir_key>(receiver.read());

        // convert the ir key to a key we can send over usb
        const auto converted_key = ir_key_to_usb(key);

        // check if we have a key we can directly send
        if (converted_key.has_value()) {
            // write the key and wait until it is done
            usb::device::write<usb, false>(converted_key.value());

            // store the key for repeats
            switch (key) {
                case ir_key::up:
                case ir_key::down:
                    previous_key = key;
                    break;
                default:
                    previous_key = ir_key::none;
                    break;
            }
        }
        else if (key == ir_key::repeat) {
            // repeat the last key if we have one
            if (previous_key != ir_key::none) {
                const auto repeat_key = ir_key_to_usb(previous_key);
                if (repeat_key.has_value()) {
                    usb::device::write<usb, false>(repeat_key.value());
                }
            }
        }
        else if (key == ir_key::released) {
            previous_key = ir_key::none;
        }
    }

    return 0;
}
