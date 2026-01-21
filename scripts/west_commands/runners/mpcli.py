# Copyright (c) 2017 Linaro Limited.
# Copyright (c) 2026, Realtek Semiconductor Corporation
#
# SPDX-License-Identifier: Apache-2.0

'''Runner for flashing bee devices with mpcli.'''

import json
import os
from pathlib import Path
from textwrap import dedent

from west import log

from runners.core import FileType, RunnerCaps, ZephyrBinaryRunner


class MPCLIBinaryRunner(ZephyrBinaryRunner):
    '''Runner front-end for mpcli.'''

    def __init__(self, cfg, port, build_dir, bin_address, chip_erase, mp_json, reset):
        super().__init__(cfg)
        self.port = port
        self.build_dir = build_dir
        self.bin_address = bin_address
        self.chip_erase = chip_erase
        self.mp_json = mp_json
        self.baud = "1000000"
        self.reset = reset
        self.elf = cfg.elf_file
        self.app_bin_file = cfg.bin_file
        self.ext_file = cfg.file
        self.ext_file_type = cfg.file_type
        self.files: list[dict[str, any]] = []

    @classmethod
    def name(cls):
        return 'mpcli'

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={'flash'}, file=True, erase=True, reset=True)

    @classmethod
    def do_add_parser(cls, parser):
        mpcli_parser = parser
        mpcli_parser.add_argument(
            '--port',
            required=True,
            type=str,
            help='Serial communication port (e.g., COM3, /dev/ttyUSB0)',
        )
        mpcli_parser.add_argument(
            '--bin-address',
            type=str,
            help='Download address(hex format, e.g., 0x8000000) for specified binary file ',
        )
        mpcli_parser.add_argument(
            '--mp-json',
            type=str,
            help=dedent('''
                        Configuration json file containing binary path and download address.
                        Example format:
                        {
                            "mptoolconfig": {
                                "port": "",
                                "baud": "",
                                "appimage": {
                                    "relativepath": "",
                                    "file": [
                                        {
                                            "id": 0,
                                            "address": "0x00801000",
                                            "name": "fw1.bin",
                                            "enable": "1"
                                        }
                                    ]
                                }
                            }
                        }
                        '''),
        )
        return parser

    @classmethod
    def do_create(cls, cfg, args):
        return MPCLIBinaryRunner(
            cfg,
            args.port,
            build_dir=cfg.build_dir,
            bin_address=args.bin_address,
            chip_erase=args.erase,
            mp_json=args.mp_json,
            reset=args.reset,
        )

    def export_to_file(self, filename: str) -> None:
        mptool_config = {
            "mptoolconfig": {
                "port": self.port,
                "baud": self.baud,
                "appimage": {"relativepath": "", "file": self.files},
            }
        }
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(mptool_config, f, indent=4, ensure_ascii=False)

    def add_file(self, address: str, name: str, id: int = 0, enable: str = "1") -> None:
        file_item = {"id": id, "address": address, "name": name, "enable": enable}
        self.files.append(file_item)

    def execute_mpcli(self, mptoolconfig_path):
        """
        Execute mpcli command with the given configuration file.
        """
        cmd_args = [
            'mpcli',
            '-c',
            self.port,
            '-f',
            mptoolconfig_path,
            '-a',
        ]

        if self.reset:
            cmd_args.extend(['-r'])

        if self.chip_erase:
            cmd_args.extend(["-E"])

        try:
            self.check_call(cmd_args)
        except Exception as e:
            self.logger.error(e.args)

    def handle_external_file(self):
        # Validate file type
        if self.ext_file_type != FileType.BIN:
            raise ValueError('Cannot flash; mpcli runner only supports bin file')

        bin_file_path = Path(self.ext_file)

        # Validate that address is provided for external files
        if not self.bin_address:
            raise ValueError('Cannot flash; --bin-address is required when file is specified')

        download_address = self.bin_address
        return bin_file_path, download_address

    def handle_build_system_file(self):
        bin_file_path = Path(self.app_bin_file)

        # Use user-provided address or derive from build configuration
        if self.bin_address:
            download_address = self.bin_address
        else:
            download_address = hex(self.flash_address_from_build_conf(self.build_conf))

        return bin_file_path, download_address

    def run_with_mp_json(self):
        """Execute mpcli using the provided json file."""
        mptoolconfig_path = self.mp_json

        # Validate that the JSON file exists
        if not os.path.isfile(mptoolconfig_path):
            log.err(f'no such json file {mptoolconfig_path}')
            return

        self.execute_mpcli(mptoolconfig_path)

    def run_without_mp_json(self):
        """
        Execute mpcli without a pre-existing json file.
        Handles both external files and build system binaries.
        """
        if self.ext_file is not None:
            bin_file_path, download_address = self.handle_external_file()
        else:
            bin_file_path, download_address = self.handle_build_system_file()

        if not os.path.isfile(bin_file_path):
            log.err(f'Cannot flash; file ({bin_file_path}) not found')
            return

        self.add_file(download_address, bin_file_path.name)

        # Export configuration to JSON file
        mptoolconfig_path = str(bin_file_path.parent / "mptoolconfig.json")
        self.export_to_file(mptoolconfig_path)

        # Execute mpcli with the generated configuration
        self.execute_mpcli(mptoolconfig_path)

    def do_run(self, command, **kwargs):
        self.require('mpcli')

        if self.mp_json:
            return self.run_with_mp_json()
        else:
            return self.run_without_mp_json()
