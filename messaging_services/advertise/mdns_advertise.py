#!/usr/bin/env python3

import dbus
import socket
import os
import time
import datetime

from messaging_services.utils.directories import META_DATA_DIR, CONFIG_DIR, STATE_DIR, UPLOAD_DIRECTORY


from messaging_services.utils.utils import get_ip, get_hostname, get_first_meta_file, get_first_file

# Define constants for Avahi
AVAHI_IF_UNSPEC = -1  # Interface UNSPEC (-1 for all)
AVAHI_PROTO_UNSPEC = -1  # Protocol UNSPEC (-1 for all)

# Directory paths

META_DATA_DIR = os.path.join(UPLOAD_DIRECTORY, 'meta_data')
CONFIG_DIR = os.path.join(UPLOAD_DIRECTORY, 'config')
STATE_DIR = os.path.join(UPLOAD_DIRECTORY, 'state')


# Ensure directories exist
os.makedirs(META_DATA_DIR, exist_ok=True)
os.makedirs(CONFIG_DIR, exist_ok=True)
os.makedirs(STATE_DIR, exist_ok=True)


class ServiceAdvertiser:
    """Advertises a service via Avahi and updates it upon configuration changes."""

    def __init__(self):
        self.bus = dbus.SystemBus()

        # Get a handle to the Avahi server
        self.server = dbus.Interface(
            self.bus.get_object('org.freedesktop.Avahi', '/'),
            'org.freedesktop.Avahi.Server'
        )

        # Create a new entry group
        self.entry_group_path = self.server.EntryGroupNew()
        self.entry_group = dbus.Interface(
            self.bus.get_object('org.freedesktop.Avahi', self.entry_group_path),
            'org.freedesktop.Avahi.EntryGroup'
        )

    def create_txt_array(self):
        """Helper function to create TXT records."""
        # Get the IP address, hostname, and first meta_data file name
        ip_address = get_ip()
        hostname = get_hostname()
        first_meta_file = get_first_meta_file()
        state_file = get_first_file(STATE_DIR)
        config_file = get_first_file(CONFIG_DIR)

        service_name = hostname
        service_type = '_http._tcp'
        domain = ''
        host = ''
        port = 8000  # Adjust as needed
        api_port = 8001  # Adjust as needed

        # Construct the TXT records
        txt_records = [
            f'ip_address={ip_address}',
            f'hostname={hostname}',
            f'port={port}',
            f'api_port={api_port}',
            f'commands_supported=upload_launch_zip:zip,upload_meta_data:json,upload_new_executable:obj,upload_config:json,upload_state:json,start_executable:command,any_file:file,launch:launch,launch_target:launch,stop:request,build_pls',
        ]

        # Convert TXT records to array of byte arrays
        txt_array = dbus.Array(
            [dbus.ByteArray(record.encode('utf-8')) for record in txt_records],
            signature='ay'
        )
        return txt_array, service_name, service_type, domain, host, port, txt_records

    def update_service(self):
        """Function to add or update the service."""
        txt_array, service_name, service_type, domain, host, port, txt_records = self.create_txt_array()

        try:
            # Reset the entry group before adding the new service
            self.entry_group.Reset()

            # Add the service
            self.entry_group.AddService(
                dbus.Int32(AVAHI_IF_UNSPEC),         # interface
                dbus.Int32(AVAHI_PROTO_UNSPEC),      # protocol
                dbus.UInt32(0),                      # flags
                service_name,                        # name
                service_type,                        # type
                domain,                              # domain
                host,                                # host
                dbus.UInt16(port),                   # port
                txt_array                            # txt records
            )

            # Commit the changes
            self.entry_group.Commit()
            print(f"{datetime.datetime.now()} - Service advertised/updated successfully:", txt_records)
        except Exception as e:
            print(f"{datetime.datetime.now()} - Failed to add/update service: {e}")

    def stop(self):
        """Stop the advertiser and cleanup."""
        self.entry_group.Reset()
        print(f"{datetime.datetime.now()} - Service advertisement stopped.")


def advertise_service():
    advertiser = ServiceAdvertiser()
    # Perform the initial advertisement
    advertiser.update_service()

    # Keep the script running
    try:
        while True:
            time.sleep(60)
    except KeyboardInterrupt:
        advertiser.stop()
        print(f"{datetime.datetime.now()} - Service advertisement stopped.")


if __name__ == '__main__':
    advertise_service()
