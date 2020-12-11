def _compile_and_load(to):
    import cppimport
    from pathlib import Path
    import sys

    # Make sure cppimport finds the library
    sys.path.append(str(Path(__file__).parent.absolute()))

    _map = cppimport.imp('_map')
    for n in dir(_map):
        to[n] = getattr(_map, n)

    # Cleanup
    sys.path.pop()


# Actually load the cpp module
_compile_and_load(globals())

# Cleanup
del _compile_and_load

