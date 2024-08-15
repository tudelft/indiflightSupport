
# packages
import yaml
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

from jinja2 import Environment, FileSystemLoader
import os
import re
import semver
from argparse import ArgumentParser, ArgumentError

# constants
BASE_DIR = os.path.join(os.path.dirname(__file__), "..")
MSGS_DIR = os.path.join(BASE_DIR, "msgs")
TEMPLATES_DIR = os.path.join(BASE_DIR, "templates")
# PI_MSG_MAX_ID = 0xFE

# beun: parse ID and PAYLOAD limits from protocol.h.j2
for line in open(os.path.join(TEMPLATES_DIR, "pi-protocol.h.j2")).readlines():
    parts = line.split()
    if len(parts) != 3:
        continue

    if (parts[0] == "#define") and (parts[1] == "PI_MSG_MAX_ID"):
        # should automatically recognize base16 due to 0x
        PI_MSG_MAX_ID = int(parts[2], 0)
    if (parts[0] =="#define" and (parts[1] == "PI_MSG_MAX_PAYLOAD_LEN")):
        PI_MSG_MAX_PAYLOAD_LEN = int(parts[2], 0)

DATATYPE_LENGTHS = {
    'uint8_t': 1,
    'uint16_t': 2,
    'uint32_t': 4,
    'uint64_t': 8,
    'int8_t': 1,
    'int16_t': 2,
    'int32_t': 4,
    'int64_t': 8,
    # 'short': 2, # may work on aarch64 with gcc: https://gcc.gnu.org/onlinedocs/gcc/Half-Precision.html
    'float': 4,
    'double': 8,
}


if __name__ == "__main__":
    # parse arguemnts 
    parser = ArgumentParser()
    parser.add_argument("config", help="Must be a y(a)ml file in the same dir as this script.")
    parser.add_argument("--protocol-h-only", required=False, action="store_true", default=False)
    parser.add_argument("--messages-h-only", required=False, action="store_true", default=False)
    parser.add_argument("--messages-c-only", required=False, action="store_true", default=False)
    parser.add_argument("--output-dir", required=False, help="Store generated headers here and not next to this script.")

    args = parser.parse_args()
    configFile = args.config

    outputPath = args.output_dir if args.output_dir is not None else BASE_DIR

    if (args.protocol_h_only+args.messages_h_only+args.messages_c_only) > 1:
        raise ArgumentError("Use none or one of --protocol-h-only or --messages-h-only or --messages-c-only, but not more")

    # for checking duplicate ids
    id_list = [] 

    # jinja2 data structure
    data = {}

    # load msgList
    if configFile not in os.listdir(BASE_DIR):
        raise ValueError(f"{configFile} does not exist in {BASE_DIR}")
    config = yaml.load(open(os.path.join(BASE_DIR, configFile), 'r'), Loader)

    # parse version number string
    config['version'] = {}
    version = semver.Version.parse(config['protocol_version'])
    config['version']['major'] = version.major
    config['version']['minor'] = version.minor
    config['version']['patch'] = version.patch

    # check id range and duplicates
    for _, msg in config['include_messages'].items():
        if msg['id'] in id_list:
            raise ValueError(f"Your {configFile} contains messages with duplicate ids!")
        id_list.append(msg['id'])
        if (msg['id'] < 0x00) or (msg['id'] > PI_MSG_MAX_ID):
            raise ValueError(f"Your {configFile} contains messages with ids outside the range 0x00 and PI_MSG_MAX_ID ({PI_MSG_MAX_ID:#x})!")

    #config['max_id'] = max(id_list)

    # load definition of all required messages and calculate payload length
    msgs = []
    for msgNAME in config['include_messages'].keys():
        print(f"Reading file {msgNAME}.y(a)ml")
        msgCandidates = [x for x in os.listdir(MSGS_DIR) \
                        if re.compile(rf'{msgNAME}.y[a]?ml').match(x)]

        if len(msgCandidates) != 1:
            raise ValueError(f"Your {configFile} requires you to provide exactly one file {msgNAME}.yaml or {msgNAME}.yml in {MSGS_DIR}")

        # load yaml definition
        msgDefinition = yaml.load(
            open(os.path.join(MSGS_DIR, msgCandidates[0]), 'r'),
            Loader)

        # parse name
        msgDefinition['nameSNAKE_CAPS'] = msgNAME
        msgDefinition['nameCamelCase'] = \
            msgNAME.replace("_"," ").title().replace(" ","")

        # calculate payload length in bytes
        msgDefinition['payloadLen'] = 0
        for field, dtype in msgDefinition['fields'].items():
            msgDefinition['payloadLen'] += DATATYPE_LENGTHS[dtype]

        if msgDefinition['payloadLen'] > PI_MSG_MAX_PAYLOAD_LEN:
            raise ValueError(f"Sum of payload bytes of message definition {os.path.join(MSGS_DIR,msgNAME)}.y(a)ml exceed PI_MSG_MAX_PAYLOAD_LEN ({PI_MSG_MAX_PAYLOAD_LEN:#x})")

        # find longest field string (for pretty message printing)
        msgDefinition['maxFieldStringLen'] = \
            max(len(item) for item in msgDefinition['fields'].keys())

        msgs.append(msgDefinition)

    # combine to single data structure and give debug output
    data['config'] = config
    data['msgs'] = msgs
    print(f"Data structure read: {data}")

    # Setup jinja2
    env = Environment(loader = FileSystemLoader(TEMPLATES_DIR),
                      trim_blocks=True,
                      lstrip_blocks=True,
                      )

    # Generate headers!
    # protocol: fill in global mode and version number
    # messages: generate all message data-types, packing and parsing logic
    protocol_h_template = env.get_template('pi-protocol.h.j2')
    messages_h_template = env.get_template('pi-messages.h.j2')
    messages_c_template = env.get_template('pi-messages.c.j2')

    if not args.messages_h_only and not args.messages_c_only:
        filename = os.path.join(outputPath, "pi-protocol.h")
        with open(filename, 'w') as protocol_header:
            print("Writing out protocol header...")
            protocol_header.write(protocol_h_template.render(data))

    if not args.protocol_h_only and not args.messages_c_only:
        filename = os.path.join(outputPath, "pi-messages.h")
        with open(filename, 'w') as messages_header:
            print("Writing out messages header...")
            messages_header.write(messages_h_template.render(data))

    if not args.messages_h_only and not args.protocol_h_only:
        filename = os.path.join(outputPath, "pi-messages.c")
        with open(filename, 'w') as messages_source:
            print("Writing out messages source...")
            messages_source.write(messages_c_template.render(data))
