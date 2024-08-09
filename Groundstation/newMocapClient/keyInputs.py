import curses

# HID codes for various keys
hid_codes = {
    'a': 0x04, 'b': 0x05, 'c': 0x06, 'd': 0x07, 'e': 0x08,
    'f': 0x09, 'g': 0x0A, 'h': 0x0B, 'i': 0x0C, 'j': 0x0D,
    'k': 0x0E, 'l': 0x0F, 'm': 0x10, 'n': 0x11, 'o': 0x12,
    'p': 0x13, 'q': 0x14, 'r': 0x15, 's': 0x16, 't': 0x17,
    'u': 0x18, 'v': 0x19, 'w': 0x1A, 'x': 0x1B, 'y': 0x1C,
    'z': 0x1D, '0': 0x27, '1': 0x1E, '2': 0x1F, '3': 0x20,
    '4': 0x21, '5': 0x22, '6': 0x23, '7': 0x24, '8': 0x25,
    '9': 0x26, ' ': 0x2C, '\n': 0x28, '\b': 0x2A, '\t': 0x2B,
    'ESC': 0x29, 'LEFT': 0x50, 'UP': 0x48, 'RIGHT': 0x4D, 'DOWN': 0x50
}

def main(stdscr):
    # Clear screen
    stdscr.clear()
    stdscr.addstr("Press keys to see their HID codes. Press ESC to exit.\n")
    stdscr.addstr("...\n")
    stdscr.addstr("...\n")
    stdscr.addstr("...\n")

    while True:
        key = stdscr.getch()  # Wait for a key press
        if key == 27:  # ESC key
            break

        # Clear the previous output
        stdscr.move(4, 0)
        stdscr.clrtoeol()  # Clear the line

        # Check if the key is an ASCII character
        if 32 <= key <= 126:  # Printable characters
            char = chr(key)
            if char in hid_codes:
                stdscr.addstr(f"HID Code for '{char}': 0x{hid_codes[char]:02X}\n")
        elif key == curses.KEY_LEFT:
            stdscr.addstr("HID Code for Left Arrow: 0x{:02X}\n".format(hid_codes['LEFT']))
        elif key == curses.KEY_RIGHT:
            stdscr.addstr("HID Code for Right Arrow: 0x{:02X}\n".format(hid_codes['RIGHT']))
        elif key == curses.KEY_UP:
            stdscr.addstr("HID Code for Up Arrow: 0x{:02X}\n".format(hid_codes['UP']))
        elif key == curses.KEY_DOWN:
            stdscr.addstr("HID Code for Down Arrow: 0x{:02X}\n".format(hid_codes['DOWN']))
        elif key == 8:  # Backspace key
            stdscr.addstr("HID Code for Backspace: 0x{:02X}\n".format(hid_codes['\b']))
        elif key == 9:  # Tab key
            stdscr.addstr("HID Code for Tab: 0x{:02X}\n".format(hid_codes['\t']))
        elif key == 10:  # Enter key
            stdscr.addstr("HID Code for Enter: 0x{:02X}\n".format(hid_codes['\n']))

        stdscr.refresh()

curses.wrapper(main)