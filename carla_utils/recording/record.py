from contextlib import contextmanager
from ctypes import c_uint8, c_uint16, c_uint32

__all__ = ['record']


@contextmanager
def record(client, filename):
    from tempfile import NamedTemporaryFile
    from zlib import compress
    from time import sleep
    try:
        m = client.get_world().get_map()
        map_name, map_data = m.name.encode(), compress(m.to_opendrive().encode())
        rec_file = NamedTemporaryFile('rb')
        client.start_recorder(rec_file.name, additional_data=True)
        yield
    finally:
        # allow carla to finish writing, TODO: Consider a better way to handle this
        client.stop_recorder()
        sleep(1)
        with open(filename, 'wb') as f:
            # Read and write the header (its variable length, thank you Carla!)
            hdr = rec_file.read(4)
            magic_len = c_uint16.from_buffer_copy(hdr[-2:]).value
            hdr += rec_file.read(magic_len+10)
            map_name_len = c_uint16.from_buffer_copy(hdr[-2:]).value
            hdr += rec_file.read(map_name_len)
            f.write(hdr)

            # Read and write entire packets to avoid file corruption
            while True:
                hdr = rec_file.read(5)
                if len(hdr) < 5:
                    break
                sz = c_uint32.from_buffer_copy(hdr[-4:]).value
                f.write(hdr)
                f.write(rec_file.read(sz))

            # Save the map using a custom packet tag=250
            f.write(c_uint8(250))
            f.write(c_uint32(len(map_data)+len(map_name)+2))
            f.write(c_uint16(len(map_name)))
            f.write(map_name)
            f.write(map_data)
