# pi-protocol
minimalist message protocol to exchange messages between betaflight and RPi via UART

## The protocol

A frame consists of:
```
| STX | MSG_ID | PAYLOAD_BYTE_1 | PAYLOAD_BYTE_2 | ... | PAYLOAD_BYTE_N |
```

`STX` bytes in `MSG_ID` and `PAYLOAD` are escaped by the sequence `ESC STX_ESC` and `ESC` bytes are escaped with `ESC ESC_ESC`. This is simple and relatively robust, but causes worst case overhead of `100%`, if every payload byte is either `STX` or `ESC`.

There are no checksums for speed and simplicity, so probably enabling the UART parity bit is a good idea, if available. On the receiving side, character-loss should be handled conservatively, such that after any inconsistency the incomplete payload is not committed and the parser waits for the next STX byte.

## Working principle of the generation of the protocol

- The `.yaml` files inside `./msgs/` define message fields and the filename defines the message name. The filename (and, as a result, the message name) should be formatted as all-caps-snake-case, like `ALL_CAPS_SNAKE_CASE.yaml`.
- A configuration `.yaml` file in the root of this repo defines which messages should be included and if they are to be received, sent or both (`RX`, `TX` or `RXTX`). Additionally, the `global_mode` option can be set to `RX` or `TX` instead of `RXTX` if it is known that all messages are `RX` or `TX`-only. In that case, the mode flags of each message have no effect.
- Python generates `c`-headers form `jinja2`-templates and define the message serialization ("packing") and parsing for receiving (packing for serial sending, parsing for serial reading).
- The resulting headers depend only on some standard libraries: `string.h`, `stdint.h` and `stdbool.h`. For testing and debugging output also `stdio.h`, of course.

## Generation

### Pre-requesites

Tested on Ubuntu 22.04. Install `make`, `python3`, `pip` and then the required python modules for generation
```bash
sudo apt install make python3 python3-pip
pip install -r python/requirements.txt # or try with sudo pip, depending on your venv setup
```

### Generate!

Edit the `yaml` files to your liking, they should be self-explanatory. You can copy the `config.yaml` and give it a different name. Then generate the headers:

```bash
make clean && make CONFIG=config.yaml
```

### Testing

You need `gcc` installed, then do

```bash
make clean && make tester CONFIG=config.yaml && ./tester
```

## Usage in your project

Look at `tester.c`. Especially note how the `dummyWriter()` is passed to `piSerialWrite()`. For `TX`, this is the function that needs to be modified/replaced to actually get the bytes on the transmission line, byte-for-byte.
