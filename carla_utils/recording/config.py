"""
  Configuration file management
  If you want to create a new configuration create a class with type decorators, and register it

    @Configuration.register('foo')
    class Foo(Settings):
        class Bar(Settings):
            name: str
            id: int = 0

        name: str = Required
        number: int
        bars: list[Bar]

  This will load the following yaml file

    foo:
        name: "hi"
        bars:
          - name: "mo"
            id: 1
          - name: "mi"
            id: 2

"""
from typing import get_type_hints, get_origin, get_args
from collections.abc import Iterable
from logging import debug
from carla_utils.util import Printable

__all__ = ["Configuration", "Required", "Settings"]


class Configuration:
    _all = {}

    @staticmethod
    def register(name, repeated=False):
        def _wrapper(cls):
            Configuration._all[name] = (cls, repeated)
            cls.__str__ = Printable.__str__
            return cls
        return _wrapper

    @staticmethod
    def _find(name):
        return Configuration._all[name][0] if name in Configuration._all else None

    @staticmethod
    def _is_repeated(name):
        return Configuration._all[name][1] if name in Configuration._all else None

    @staticmethod
    def _has(name):
        return name in Configuration._all

    @staticmethod
    def from_file(*fns):
        from pathlib import Path
        settings = {}
        for fn in fns:
            fn = Path(fn)
            if fn.name.endswith('.json'):
                import json
                with open(fn, 'r') as f:
                    settings.update(json.load(f))
            elif fn.name.endswith('.yaml'):
                import yaml
                with open(fn, 'r') as f:
                    settings.update(yaml.load(f))
            else:
                raise ValueError('Unknown file extension for configuration "%s"' % fn.name)
        return Configuration.from_dict(settings)

    @staticmethod
    def from_dict(o):
        r = Configuration()
        for k, v in o.items():
            tpe = Configuration._find(k)
            assert tpe is not None, 'No configuration found for setting {!r}'.format(k)
            if Configuration._is_repeated(k):
                assert isinstance(v, Iterable), 'Expected repeated type {!r} got {!r}'.format(k, type(v))
                setattr(r, k, [tpe(**i) for i in v])
            else:
                setattr(r, k, tpe(**v))
        return r


class Required:
    pass


class Settings(Printable):
    def __init__(self, **kwargs):
        th = get_type_hints(self.__class__)
        for k, v in kwargs.items():
            if k in th:
                setattr(self, k, convert_to(v, th[k]))
            assert hasattr(self, k), 'Unknown settings value "%s"' % k
        for k in vars(self.__class__):
            assert getattr(self, k) != Required, "Required configuration value {!r} not specified".format(k)


def convert_to(v, tpe):
    # Make sure v is of decorated type tpe
    if tpe is None:
        return v
    if get_origin(tpe) == list:
        assert isinstance(v, list), 'Setting expected type {!r} got {!r} ({!r})'.format(tpe, type(v), v)
        nested_tpe = get_args(tpe)
        return [convert_to(i, *nested_tpe) for i in v]
    if get_origin(tpe) == dict:
        assert isinstance(v, dict), 'Setting expected type {!r} got {!r} ({!r})'.format(tpe, type(v), v)
        nested_tpe = get_args(tpe)
        return [convert_to(i, *nested_tpe) for i in v]
    if issubclass(tpe, Settings):
        assert isinstance(v, dict), 'Setting expected type {!r} got {!r} ({!r})'.format(dict, type(v), v)
        return tpe(**v)
    if isinstance(tpe, type):
        if not isinstance(v, tpe):
            debug('Configuration type check failed for value {!r} and type {!r}'.format(v, tpe))
        try:
            return tpe(v)
        except ValueError:
            raise ValueError('Setting expected type {!r} got {!r} ({!r})'.format(tpe, type(v), v))
    raise ValueError('Setting decorator not a type {!r}'.format(tpe))
